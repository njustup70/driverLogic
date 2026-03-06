import asyncio
from typing import TypeVar, Generic, Optional, Generator, Any

# 1. 定义泛型变量 T，它代表你存入 value 的那个类（例如 RosBridgeNode）
T = TypeVar("T")

class AsyncVariable(Generic[T]): # 👈 继承 Generic[T] 是补全的关键
    """
    针对类型补全优化的异步变量类。
    继承 Generic[T] 后，IDE 就能追踪 await 之后返回的具体对象类型。
    """
    def __init__(self, value: T):
        self._value: T= value
        self._event: Optional[asyncio.Event] = None
        #这里不一定在同一个线程里面,所以需要线程安全的事件循环访问
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def _get_event(self) -> asyncio.Event:
        if self._event is None:
            self._event = asyncio.Event()
        return self._event

    @property
    def value(self) -> T:
        return self._value

    @value.setter
    def value(self, new_value: T):
        self._value = new_value
        # 捕获当前正在运行的异步事件循环
        if self._loop is None:
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                pass
        
        if self._loop and not self._loop.is_closed():
            # 使用 call_soon_threadsafe 确保即使在外部线程赋值，也能在异步线程唤醒
            self._loop.call_soon_threadsafe(self._notify)

    def _notify(self):
        # 唤醒所有等待这个事件的协程
        if self._event:
            self._event.set()
            # 注意：clear 放在这会导致所有 await 者被唤醒
            self._event.clear()

    # 2. 关键：明确标注返回类型为 Optional[T]
    def __await__(self) -> Generator[Any, None, Optional[T]]:
        """
        允许使用 'node = await var' 获取更新。
        明确标注了返回类型，从而激活 IDE 补全。
        """
        if self._loop is None:
            try:
                self._loop = asyncio.get_running_loop()
            except RuntimeError:
                pass
        ''' 等效于await self._get_event().wait()，但是这里是普通函数,所以不能直接await,需要yield from '''
        yield from self._get_event().wait().__await__()
        
        return self._value
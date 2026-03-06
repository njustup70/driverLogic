import asyncio
from typing import TypeVar, Generic, Optional, Generator, Any, Callable, cast, overload

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

    # 2. 关键：明确标注返回类型为 T
    def __await__(self) -> Generator[Any, None, T]:
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


class AsyncValueProxy(Generic[T]):
    """
    对外暴露“像原对象一样可访问 + 可 await”的代理。
    - obj.attr 访问底层 value 的属性
    - await obj 等待下一次更新并返回最新 value
    """

    def __init__(self, async_var: AsyncVariable[T]):
        object.__setattr__(self, "_async_var", async_var)

    @property
    def value(self) -> T:
        return self._async_var.value

    @value.setter
    def value(self, new_value: T):
        self._async_var.value = new_value

    def __await__(self) -> Generator[Any, None, T]:
        return self._async_var.__await__()

    def __getattr__(self, name: str):
        return getattr(self._async_var.value, name)

    def __setattr__(self, name: str, value):
        if name == "_async_var":
            object.__setattr__(self, name, value)
            return
        current = self._async_var.value
        setattr(current, name, value)
        self._async_var.value = current

    def __getitem__(self, key):
        return self._async_var.value[key]

    def __setitem__(self, key, value):
        current = self._async_var.value
        current[key] = value
        self._async_var.value = current

    def __repr__(self) -> str:
        return repr(self._async_var.value)


class AsyncProperty(Generic[T]):
    """
    类似 C# 自动属性的写法：
    - 直接用 instance.attr 读写
    - 需要 await 更新时，通过 descriptor.get_async(instance) 获取 AsyncVariable
    """

    def __init__(self, default_factory: Callable[[], T]):
        self._default_factory = default_factory
        self._storage_name = ""
        self._proxy_name = ""

    def __set_name__(self, owner, name: str):
        self._storage_name = f"__async_property_{name}"
        self._proxy_name = f"__async_property_proxy_{name}"

    def _ensure_var(self, instance) -> AsyncVariable[T]:
        async_var = cast(Optional[AsyncVariable[T]], getattr(instance, self._storage_name, None))
        if async_var is None:
            async_var = AsyncVariable(self._default_factory())
            setattr(instance, self._storage_name, async_var)
        return async_var

    def _ensure_proxy(self, instance) -> AsyncValueProxy[T]:
        proxy = cast(Optional[AsyncValueProxy[T]], getattr(instance, self._proxy_name, None))
        if proxy is None:
            proxy = AsyncValueProxy(self._ensure_var(instance))
            setattr(instance, self._proxy_name, proxy)
        return proxy

    @overload
    def __get__(self, instance: None, owner: type) -> "AsyncProperty[T]":
        ...

    @overload
    def __get__(self, instance: object, owner: type) -> AsyncValueProxy[T]:
        ...

    def __get__(self, instance, owner) -> Any:
        if instance is None:
            return self
        return self._ensure_proxy(instance)

    def __set__(self, instance, value: T):
        self._ensure_var(instance).value = value

    def get_async(self, instance) -> AsyncVariable[T]:
        return self._ensure_var(instance)


def async_property(default_factory: Callable[[], T]) -> AsyncProperty[T]:
    return AsyncProperty(default_factory)
'''
Async魔法部分,请勿修改
请勿修改
请勿修改
请勿修改
目前只能通过在类内定义属性来达成最简单的使用
'''
'''
使用方法1:调用AsyncVariable
var = AsyncVariable(Odom)
var.x=1
var.value=Odom(1,2,3) #整个重新复制需要调用value属性
var.value.x =1 #修改属性后需要再赋值一次触发更新,var.value=var.value
使用方法2:调用AsyncProperty,需要在类内定义属性
class TFManager:
    baseLinkOdom = async_property(Odom)
tfManagerInstance = TFManager()
tfManagerInstance.baseLinkOdom = Odom(1,2,3) #直接赋值就行了,不需要调用value属性
tfManagerInstance.baseLinkOdom.x=1 #支持局部更新触发
'''
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
        self._value: T = value
        self._event: Optional[asyncio.Event] = None
        #这里不一定在同一个线程里面,所以需要线程安全的事件循环访问
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def _get_event(self) -> asyncio.Event:
        if self._event is None:
            self._event = asyncio.Event()
            self._event.set()  # ✅ 修复：初值设置事件，避免首次 await 无限阻塞
        return self._event
        
    #只有整个重新赋值才会触发更新,修改value的属性不会触发更新,所以需要在外面修改完属性后再赋值一次,比如baseLink.value=baseLink.value
    # （注：由于下方合并了原代理的 __setattr__ 魔法，现在直接修改属性也会自动触发更新了！）
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
        event = self._get_event()
        event.clear()  # ✅ 修复：先清除（复位）
        event.set()    # ✅ 修复：再设置（唤醒等待者）

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

    # ==========================================
    # 以下为原 AsyncValueProxy 吸收进来的代理魔法方法
    # ==========================================

    def __getattr__(self, name: str):
    #在找不到原生属性时才会调用__getattr__,所以这里直接访问底层对象的属性就行了,不需要担心死循环
        return getattr(self._value, name)

    def __setattr__(self, name: str, value: Any):
        #在设置属性时，如果是 AsyncVariable 自身的内部属性，就直接设置；否则修改底层对象的属性，并触发更新
        # 放行 AsyncVariable 自身的内部属性
        if name in ("_value", "_event", "_loop","value"):
            super().__setattr__(name, value)
            return
        # 修改底层对象的属性，并触发更新
        current = self._value
        setattr(current, name, value)
        self.value = current 
        
    def __repr__(self) -> str:
        return repr(self._value)

class AsyncProperty(Generic[T]):
    """
    类似 C# 自动属性的写法：
    - 直接用 instance.attr 读写
    - 需要 await 更新时，通过 descriptor.get_async(instance) 获取 AsyncVariable
    """

    def __init__(self, default_factory: Callable[[], T]):
        self._default_factory = default_factory
        self._storage_name = ""

    def __set_name__(self, owner, name: str):
        self._storage_name = f"__async_property_{name}"

    def _ensure_var(self, instance) -> AsyncVariable[T]:
        async_var = cast(Optional[AsyncVariable[T]], getattr(instance, self._storage_name, None))
        
        if async_var is None:
            async_var = AsyncVariable(self._default_factory())
            setattr(instance, self._storage_name, async_var)
        return async_var

    @overload
    def __get__(self, instance: None, owner: type) -> "AsyncProperty[T]":
        ...

    @overload
    def __get__(self, instance: object, owner: type) -> AsyncVariable[T]:
        ...

    def __get__(self, instance, owner) -> Any:
        if instance is None:
            return self
        # 直接返回 AsyncVariable，砍掉了中间的 proxy 层
        return self._ensure_var(instance)

    def __set__(self, instance, value: T):
        self._ensure_var(instance).value = value

    def get_async(self, instance) -> AsyncVariable[T]:
        return self._ensure_var(instance)


def async_property(default_factory: Callable[[], T]) -> AsyncProperty[T]:
    return AsyncProperty(default_factory)
# // .............................................'RW#####EEEEEEEEEEEEEEEEEEEEEEEEWW%%%%%%N%%%%%%NW"...........
# // ............................................/W%E$$$$EEEE######EEEEEEEEEEEEEEEE%%@NN@@$@@N%%%%N%]~`........
# // ........................................i}}I&XIIYYXF&R#E$$$$$EEE##EEEEEEEEEEEE$N$#$K1:!YW@N%%%%@N$KY]+";..
# // .....................................!>>li!"~~~'~~~~~!"i/1lIFK#E$$$EEEEEEE$$EEE%I::.....,]E@@@NNN@M$E$R>..
# // ....................................+1"""i>"""""!~''''~~~!!~~!>/]Y&#$$$EEEEWWEEE$F,.......:>IRE$#&I/>'....
# // ...................................;*lX&NM@@NW$#RFIl1i"!~~"">>!~~~!i}Y&#$W$EW%$EEMi...........::...'l1....
# // ]}/+>~,............................,*YRNNNN@@MMMMMMMM@WRF*1>!~"!~!!~~!>+1IK$W%%W%1.................!*+....
# // FFF&K&FYYYI]/"'`....................!K%W$$$$$$$EEEEEEE$W%%%WE&I]+!~~~!">"~~i*#%@#...................';....
# // }}}}}}]l*XR#$WWERXl/!,:........,>>i/YK&&&&KKKKRR##EE$$$$$EEEE$$$EKYl/>!'~!"!+]IRNI..................'':...
# // lllll]]]]}IYYXFK#W%N%$RFl+~`..`X/>>>!~~~~~!!"""">>ii+/}*YXK#EE$$WWWW$#Fl+"'~+**]*FI"................>i....
# // ]]]]]]]]]]YXXXXYYXFRE$WW%%W#FlXl;!">+//i">"~'''''~!""!~~~!""i/1]*YFR#$%%WE&l/1]**lI&!.............>]]ll~..
# // ]]]]]]]]]*XXXXXXXXYYX&R$$EE$$WWRR#WWWWW$E##KXI*1>!~!!""""!!!~~~'~~!!"+}I&R$NNWKYll*E"............"}/,~I&'.
# // ]]]]]]]]lYXXXXXXXXXXXYYXKE$$E#E$$$$$$$$$$$$$$WWW$#X}1+>>""!''''~!!">""""""/]Y#W%$FRY............./+,.~lF>.
# // ]]]]]]]]YXXXXXXXXXXXXXXXYYFKEW$E#EEEEEEEEEEEEEEEE$$WWW$$E#RFYl/+i!''!>"!!~!i]]]*XR#1'............!I/!]XI`.
# // ]]]]]]]IXXXXXXXXXXXXXXXXXXYYYFE%WEEEEEEEEEEEEEEEEEEEEEEE$$$%%NN%$EKY]+i"!!"ilII*l]lXK/.:..........;+1/>:..
# // ]]]]]]IXYXXXXXXXYYYYXXXXXXXYYR$RK$%$EEEEEEEEEEEEEEEEEEEEEEEE##EE$%NNNWE#R&&XI**llll]Y*.......::`,,`:::....
# // ]]]]]*XXXXXXXXXXYYYYYYYXXXY&$#I/>/YE%$EEEEEEEEEEEEEEEEEEEEEEEEEEEE#EE$$WW$$W$ER&Y**]&}~+]IFRE$WW%%%%W$$#KX
# // ]]]]lYXXXXXXXYYYYYYYYYYXXYK#I/ii+i>lYKWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$$WW$$#@%NMMM@@NNNN%%NNNNNN@@
# // ]]]]YXXXXXXYYYYYYYYYYYXYYKX1iiiii+l1>i}KWWE#EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$W%@N%%%%%%%%%%%%%%%%%%%
# // ]]]*XXXXXXXXXXXXYYYYYXYY&*++iiii+]+>++>11X$%$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N@@NNNN%%%%%%NNN%%
# // ]]]YXXXXXXXYYYYYYXXYXYX&}i+iiiii1+iiii+*>>+*RWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N%W$$$$$$$$$WW%%
# // ]]*XXYYYYYXX&K#$&YXXXXK}>+iiiii++iiii+FI>+i>>}F$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // }}IYYXFK#E%N%%NEYXXXYK}>iiiiiiiiii+>1I}]>iiii>"+*R$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // &XK#EWN@@%#YWN$YYXXY&l>iiiiii++iii>}I"il>iiiiiii"+1IR$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // $NN@N$&}!`.*NWXYXXYFI1/ii++i"!i>+>}l"!i]"iiiiiiii/i>/1IEW$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // "Y&l>,.:~1F@%KYXXXXF}Yi+i"',.';:,1]"+!'/;i">iiii>/i/1"1]IIX#$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // ......*@M@%RFIYYXYF1F}!'`::::!`."]""~'!]~"":~"iii/i}/+F***>i*YF#E$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // ......`+FWNWERFXYXl+Y`::::::,".!}~,:.:.~',*:::,'!+/}!/]!>l/"}X>i1lXRE$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
# // .........;/XEN@N%Wi&].`:::`:!',]/1i~,::`>`}>.`::.~1""1;::~1"+]li">>/llXRE$$$$$EE##EEEEEEEEEEEEEEE####EEEE$
# // ............:'+lFK}N+:`:::`:!!&%NW$W$&]~,'!/~.``.~!.!''>+/IY>]lX/""11>ii+1lY&#$WWWW$$$$$E$$$$$$$WWW%%%W$#&
# // .................>i~~,`:::`,}#M#}"'F%$W$}`;'!;.:`,','1XFK@@@RF@@@&~~~,!>ii>>+i/}lIXF&KR#$R#RRRKK&XIl1i!'`.
# // ................`+'.~'`::`:`}$X`.::"&KFK&,:`'~!:`,'`~!;.;X&FKK$&l##l'::`;!>+/+i>>>>>>ii>I!.:..............
# // ...:;..`!/:.....;i;."!`::`:;I+~.:.!E&FW#K':'',!"''';...;lYR#K&#K;"1#]~`::,:>i~i++++iii/}++................
# // ...;Y.;/Y`......'>;.>+,::`.~E]:::.'K/"l}i`:`:::`;;'!;`.iK/}%&lRI.`1*'``:,,:+"~,,~"i++i1#+},...............
# // ...:,>/.......~"'.;l;:``:'Y]'```./+~'';::``::::::`,`:~/~~i"!/'.:i;:`:,,:/+;''.::,'!"**>li...............
# // ....:,.;;.......~"':.>i:`:',+l>;'';'";;,``:`:::::::::`,`'~~~~+":`~~```;;'>li!;`::::,.`Xi.1l`..............
# // .......'];......`i;;.:I'::'I>1>'~~'';;,,```::::::::```,;;''~~''~~'`,'>>>>'/"~`,::::,`,X".`l+..............
# // ........~~......."'~`/>+:::lll";'';;;,,,``:``;~::::```,;;'''~!>>"!!>ii!;:,~,'+!::::`;,Y~..,Y'.....::......
# // .......:,.........!~/!.~+`.>]*>:,,,,,,````:`;;;`::````,,,,;;;~!'1/"!;`:::!~+]+"`::`:;;*~...~l.....`>,.....
# // ........>..........>i!!++!'`/1Ii`::```````:::::`:::::`````,,,,`'i``:::::;]}/iii,:`::`!*~....>".....`/`....
# // ........,`........;>.'>"~.i++]/ll+'`::::::``````::::::````::..,+```:`:`:"+ii+++':`:`:;Fi.....i`.....;+....
# // ........`,.......`+:~!.;':i++/+i/}}1+>~;`::.....::::::...:,~i]X~`,:`:`:;/+++++1!:`::`.]I.....`"......+;...
# // ........:~......:i,~'..";,ii+/+++++/1}]]}1/i>"!!;,,,,,>}lII**Y>`,:`:;,,/++++/+1+`:::`:,F,.....~,.....'>...
# // ................"~'`..:1,,+i1+++//////}111}}}IY$K">>>!*NFl&X]>,,:`.'~`*]++++//+}':`:``.>1.....:".....'i...
# // ...............~i':...!1.'+//+++//+/+]l+1]lIF]/Kl"">>+11>"1&i``::`!~;]I]++++//i1i:`::~;./;.....",....!;...
# // ..............,1;.....]~.~+}++++/+///*]Y&F$Kl+}1!i++}1+i"11'`:::;!~i*]l]+//+/1++/,:`:~i``/.....'".........
# // .............:]~....."}.:"}/+++////i1XRRYF*]lFKY/lI/`;,:"+~!:::,!"l&XYFY++/+/]++/~:`:~+".~i....;/.........
# // .............+>.....`*;::/}++++//+/*#El}FIl*F&X]I*IX+`;1}'i::,~>+FYIX&#%#/++1]i++>:`:~++;.+'...~l.........
# // ............;}......+!.:'l+++++1+/#N@/'i#F1!1]*"l*I*]+"+l}1+i+i>iRRE$$$ENIi+1]++//;::!++":`i.:.1].........
# // ............/~....."i.`.+1i+++/1i*WW&~!1Wi`,+i/]Il>'`:.'I*Y>!>}FE$$EEEE#%*i+}}++1}!:`>+i+,.!~."Ii.........
# // ............]'....'/.`;`1+++++//i&EW*~~YF'>+}]1//i"`.:''.Y+iY#$$EEEEEEEEW]i+}/++/*i`,++i/]::>+*l,.........
# // ...........`l"...`/`:"~'/}/+++//YWEW*!+*+>iiI]]/">>i;,!`.lX$$EEEEEEEEEEEWN*+1++++Y1;'+i+i*+.~l}~..........
# // ...........`l/,..+;.,+'~]*+//+1EW$E$X+1"""iY&i1l>"""ii+`,EWEEEEEEEEEEEEEE$W1++++i*]'>+i++//~.+;...........
# // ............]}/`"".:!+~!I}+//iY%#$$EX1~""iFI*~i1]"">!/]`"%EEEEEEEEEEEEEEE$$1i++++/1"+++++};".!;...........
# // ............'Y/1+.`:>+>/]1+//iXW#E$$**X>+F*/";">]l!"+&%I+WE$$$$$$$E$$WW%N%}i++++/>i+i++++]:;;,>...........
# // .............~l}`::;i+i]i}+/+l@%$$E$&XYYY}!1,.!i>XIYX#N$REWEEEEEEEEEEEEE$N1+/+++/'iiiiii/1.`>:+...........
# // ..............,i::;}i+i],*+++$@$EE$E%&']!~"+;~;>!"*}>$$$$EEE$EEEEEEEEEEE#%Fi//+1~'/i+++il>..>`i`..........
# // ..............';:./]i++}.}li]NEEEE$W$}:i]+i!;~;i>i>,>%F/*$EEWWEEEEEEEEEEEE%Xi+//./+i++++*`..";i`..........
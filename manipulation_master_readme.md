# Manipulation Masterの使い方  
物体認識・把持を管理するマスター  
Actionを通してobject_recognizerで物体認識、object_grasperで物体把持を行う  
  
# Usage  
競技用マスターからはサービスで立ち上げます  
  |Name|Type|Input|Output|  
  |:---|:---|:---|:---|  
  |/manipulation|ManipulateSrv|String型の`target`|Bool型の`result`|  

使用している型は独自srvなのでimportしてください  
MiniPC(NUC):  
    **`from mimi_common_pkg.srv import ManipulateSrv`**  
Jetson:  
    **`from mimi_manipulation_pkg import ManipulateSrv`**  

例としてこんな感じ  
    **`rospy.ServiceProxy('/manipulation', ManipulateSrv)`**  

# Input and Output  
把持したい物体の名前(darknetに表示される名称)をString型で引数として呼び出して下さい  
物体の名前を`'any'`とした場合は把持可能な物体を適当に把持します  

把持に成功で`True`, 失敗で`False`を返します  
  
# Index  
### [パッケージの説明](https://github.com/HappyTatsuhito/mimi_manipulation_pkg)  
> パッケージの概要、立ち上げ等の説明  
### Manipulation Masterの使い方  
> 物体認識から把持までの一連のタスクを行うマスターの使い方  
### [Recognizerの使い方](/recognizer_readme.md)  
> 物体認識を補助するモジュールの使い方  
### [Grasperの使い方](/grasper_readme.md)  
> アームの制御を行うモジュールの使い方  

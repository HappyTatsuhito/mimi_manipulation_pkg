# Overview  
物体認識・把持を管理するマスター  
Actionを通してobject_recognizerで物体認識、object_grasperで物体把持を行う  
  
# Usage  
物体把持に必要なノードを立ち上げます(RealSenseやdarknetについては省きます)  

    $ roslaunch manipulation manipulation.launch  

競技用マスターからはサービスで立ち上げます  
  |Name|Type|Args|  
  |:---:|:---:|:---:|  
  |/manipulation|manipulation/ManipulateSrv|target(String)|  

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

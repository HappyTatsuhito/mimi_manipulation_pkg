# Config Param Description  
パラメータ調整の行い方・変数の説明  
  
# Content  
- **mimi_motor.yaml**（configに所属）  
    > motor_setup.launchで使用するyamlファイル  
      使用するモータの角速度などの設定を記述する。  
- **mimi_specification.yaml**（paramに所属)  
    > 各種ノードで使用するyamlファイル  
      ハードウェアの長さやアームの規格などmimiの仕様を記述する。  
  
# How to  
## mimi_motor.yaml  
- ### 概要  
  中身はこんな感じになってます。※一部省略  
  ```  
  m0_controller:
    ID: 0
    Return_Delay_Time: 0
    Operating_Mode: 3
    Profile_Acceleration: 0
    Profile_Velocity: 80
  m1_controller:
    ID: 1
    Return_Delay_Time: 0
    Operating_Mode: 3
        ・
        ・
        ・
  ```  
  構造としては
  ```
  モータの名前：
    モータの設定
  ```
  となっています。  
  モータの名前は自分で命名する必要がありますが、各モータの区別が付けば大丈夫です。  
  基本的には上記の5つの設定を記述しておけば問題ないと思います。  
  
- ### 変数説明  
  |名前|説明|  
  |:---:|:---|  
  |ID|DynamixelWizardで割り振ったIDを記述します。※この値でモータを判別|  
  |Operation_Mode|制御モードを指定します。角度(Joint)制御を使うので3にします。<br>他の制御方法を使用する場合は別の数値に|  
  |Profile_Velocity|モータの角速度を指定します。使用する場所に合わせて変更をおすすめします。|  
  
  他の変数は基本0なので述べません。気になる方は各自調べてもらえると助かります。  
  
  
## mimi_specification.yaml  
- ### 概要  
  mimiのアームのジョイント毎の長さやRealSenseの高さ、モータの初期角度など機体の仕様を記述します。  
  manipulation.launchにてrosparamでマスターに登録されます。  
  
- ### 変数説明  
  |名前|説明|  
  |:---:|:---|  
  |Ground_RealSense_Height|地面からのRealSenseの高さ|  
  |Ground_Neck_Height|地面からの首モータの高さ|  
  |Ground_Arm_Height|地面からのアームの取り付け部分の高さ|  
  |Shoulder_Elbow_Length|肩モータから肘モータの長さ|  
  |Elbow_Wrist_Length|肘モータから手首モータの長さ|  
  |Wrist_Endeffector_Length|手首モータからエンドエフェクタの物体が触れる部分までの長さ|  
  |Origin_Angle|アームを地面と水平にした際の各モータの角度|  
  |Gear_Ratio|各ジョイントのギア比（左からモータID順）|  
  

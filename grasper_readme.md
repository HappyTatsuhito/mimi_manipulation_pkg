# Overview  
物体把持を行うノードobject_grasperについて  
Actionを通してmanipulation_masterと通信して物体把持を行うだけでなく、アーム（サーボモータ）の制御機能が搭載されています。  
このページはその機能（モジュール）をメインに解説しています。  
  
# Usage  
モジュールの使い方について
  |Module|Communication|Name|Type|Input|Output|  
  |:---|:---|:---|:---|:---|:---|  
  |Control Shoulder|Topic|/servo/shoulder|Float64|-3.14～3.14||  
  |Control Elbow|Topic|/servo/elbow|Float64|-3.14～3.14||  
  |Control Wrist|Topic|/servo/wrist|Float64|-3.14～3.14||  
  |Control Endeffector|Topic|/servo/endeffector|Bool|True||  
  |Control Head|Topic|/servo/head|Float64|-3.14～3.14||  
  |Change Arm Pose|Service|/servo/arm|ManipulateSrv|String型の`target`|Bool型の`result`|  
  
## Control Shoulder  
肩の部分に搭載されている二つのサーボモータを制御するモジュール  
正で上、負で下  
0で肩が地面に対して水平  
  
## Control Elbow  
肘の部分に搭載されているサーボモータを制御するモジュール  
正で上、負で下  
0で肘が地面に対して水平  
  
## Control Wrist  
手首の部分に搭載されているサーボモータを制御するモジュール  
正で上、負で下  
0で手首が地面に対して水平  
  
## Control Endeffector  
手（エンドエフェクタ）の部分に搭載されているサーボモータを制御するモジュール  
正で閉、負で開  
  
## Control Head  
首の部分に搭載されているサーボモータを制御するモジュール  
正で上、負で下  
0で正面を向く
  
## Change Arm Pose  
アーム全体を制御してポーズを変えるモジュール  
入力されたデータに合わせてポーズを変える。動作を完了するとTrueを返す。    
  
機能紹介  
- 入力：`"carry"`　の場合はアームを畳む。移動するときに使用する。  
- 入力：`"give"`　の場合は把持しているものを他者に渡す。物体を渡し終える、または一定時間経過で自動的に`"carry"`の状態に戻る。  
- 入力：`"place"`　の場合は把持しているものを正面（？？[m]前方）の机に設置する。使用には条件があり、`/current_location`に現在のロケーション名をpublishする必要がある。物体を設置し終えると自動的に`"carry"`の状態に戻る。  
  
# Caution  
  
  
# Index  
### [パッケージの説明](https://github.com/HappyTatsuhito/mimi_manipulation_pkg/blob/master/README.md)  
> パッケージの概要、立ち上げ等の説明
### [Manipulation Masterの使い方](/manipulation_master_readme.md)  
> 物体認識から把持までの一連のタスクを行うマスターの使い方  
### [Recognizerの使い方](/recognizer_readme.md)  
> 物体認識を補助するモジュールの使い方  
### Grasperの使い方  
> アームの制御を行うモジュールの使い方  

# Overview  
物体認識を行うノードobject_recognizerについて  
Actionを通してmanipulation_masterと通信して  
  
# Start Up  
立ち上げはmanipulation_masterの立ち上げと同時に立ち上がります。  
  
    $ roslaunch manipulation manipulation.launch  
  
# Usage  
モジュールの使い方について
  |Module|Communication|Name|Type|  
  |:---:|:---:|:---:|:---:|  
  |Find Object|Service|/recog/find|RecognizeFind|  
  |Count Object|Service|/recog/count|RecognizeCount|  
  |Localize Object|Service|/recog/localize|RecognizeLocalize|  
  
### Find Ojbect  
物体を見つけるモジュール  
入力されたデータに合わせて物体を探す。一定時間内に見つからなければFalseを返す。  
**入力：String型のtarget　出力：Bool型のresult**  
  
機能紹介  
- 入力：`"物体の名前"`　の場合はその物体を探す  
- 入力：`"any"`　の場合は既知の把持可能物体を探す  
- 入力：`"None"`　の場合は認識可能な物（把持不可能も含む）を探す  
  
### Count Ojbect  
物体を数えるモジュール  
入力されたデータに合わせて個数を数える。個数と認識した物体のリストを返す。  
  
**入力：String型のtarget　出力：Int64型のnum, String[]型のdata**   
  
機能紹介  
- 入力：`"物体の名前"`　の場合はその物体の数と認識した物体のリストを返す  
- 入力：`"any"`　の場合は`num=0`と既知の把持可能物体を左から順に並べたリストを返す  
  
### Localize Ojbect  
物体の  
  
**入力：String型のtarget　出力：geometry_msgs/Point型のdata**

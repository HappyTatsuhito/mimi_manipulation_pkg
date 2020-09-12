# Overview  
物体認識を行うノードobject_recognizerについて  
Actionを通してmanipulation_masterと通信して  
  
# Start Up  
立ち上げはmanipulation_masterの立ち上げと同時に立ち上がります。  
  
    $ roslaunch manipulation manipulation.launch  
  
# Usage  
モジュールの使い方について
  |Module|communication|Name|Type|  
  |:---:|:---:|:---:|:---:|  
  |Find Object|Service|/recog/find|RecognizeFind|  
  |Count Object|Service|/recog/count|RecognizeCount|  
  |Localize Object|Service|/recog/localize|RecognizeLocalize|  
  
### Find Ojbect  
    物体を見つけるモジュール  
  入力されたデータに合わせて物体を探す。一定時間内に見つからなければFalseを返す。
  入力：String　出力：Bool

  機能紹介
  - 入力：`'物体の名前'`　の場合はその物体を探す
  - 入力：`'any'`　の場合は既知の把持可能物体を探す  
  - 入力：`'None'`　の場合は認識可能な物（把持不可能も含む）を探す  

### Count Ojbect  

### Localize Ojbect  

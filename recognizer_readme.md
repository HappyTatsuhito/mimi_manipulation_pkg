# Overview  
物体認識を行うノードobject_recognizerについて  
Actionを通して  
  
# Start Up  
立ち上げはmanipulation_masterの立ち上げと同時に立ち上がります。  
  
    $ roslaunch manipulation manipulation.launch  
  
# Usage  
モジュールの使い方について
  |communication|Name|Type|  
  |:---:|:---:|:---:|  
  |Service|/recog/find|RecognizeFind|  
  |Service|/recog/count|RecognizeCout|  
  |Service|/recog/localize|RecognizeLocalize|  
  

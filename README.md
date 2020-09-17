# Overview  
物体認識・把持、アーム制御などを行うパッケージ  
主な機能としては以下の３つがあります。  
- 物体認識から把持までの一連のタスクをActionで管理・実行する  
- 物体認識を補助するモジュールを備えている  
- アーム制御を行う  
  
これらを**競技マスター**から使用する方法を記述しています。  
このパッケージの通信形態の概要は以下の図のようになっています。  
  
![物体認識・把持](https://user-images.githubusercontent.com/33217285/76415666-ee49f280-63dc-11ea-93c2-8c845aeedfe9.png)  
  
# Start Up  
必要なノードは以下二つのコマンドで全て起動します。(RealSenseやdarknetについては省きます)  
  
  モータを制御するドライバの起動  
  
    $ roslaunch mimi_manipulation_pkg motor_setup.launch  
  
  本パッケージの起動  
  
    $ roslaunch mimi_manipulation_pkg manipulation.launch  
  
# Index  
### パッケージの説明  
> パッケージの概要、立ち上げ等の説明  
### [Manipulation Masterの使い方](/manipulation_master_readme.md)  
> 物体認識から把持までの一連のタスクを行うマスターの使い方  
### [Recognizerの使い方](/recognizer_readme.md)  
> 物体認識を補助するモジュールの使い方  
### [Grasperの使い方](/grasper_readme.md)  
> アームの制御を行うモジュールの使い方  

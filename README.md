# Overview  
物体認識・把持、アーム制御などを行うパッケージ  
主な機能としては以下の３つ  
- 物体認識から把持までの一連のタスクをActionで管理・実行する  
- 物体認識を補助するモジュールを備えている  
- アーム制御を行う  
  
![物体認識・把持](https://user-images.githubusercontent.com/33217285/76415666-ee49f280-63dc-11ea-93c2-8c845aeedfe9.png)  
  
# Start Up  
上の主な３つの機能に必要なノードはこのコマンドで全て起動します。(RealSenseやdarknetについては省きます)  

    $ roslaunch mimi_manipulation_pkg manipulation.launch  
  
# Index  
### [Manipulation Masterの使い方](/manipulation_master_readme.md)  
> 物体認識から把持までの一連のタスクを行うマスターの使い方  
### [Recognizerの使い方](/recognizer_readme.md)  
> 物体認識を補助するモジュールの使い方  
### Grasperの使い方  
> アームの制御を行うモジュールの使い方  

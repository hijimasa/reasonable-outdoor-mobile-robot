# 廉価版屋外向け移動ロボットプラットフォーム

このリポジトリは、つくばチャレンジや中之島チャレンジで個人参加しやすい廉価な屋外向け移動ロボットプラットフォームを実現するために立ち上げたものです。
できるだけ廉価になるように設計してみたのですが、材料費は20万円程度となってしまったので、個人が手軽に始められるかどうかはちょっと怪しいです。
本リポジトリが他の参加者の一助となれば幸いです。

![ロボット外観](figs/reasonable_robot.png)

# リポジトリの構成

- CAD

  様々な3DCADツールで利用しやすいSTEPファイルとROSでシミュレーションする際に利用できるSTLファイルを用意しています。
  
- BoM

  ロボット製作に必要となった部品表とかかった金額のメモです。参考までに。
  
- tools
  
  本リポジトリのロボットを活用するためのツールとして、モータのID変更やコントローラに関するArduinoプロジェクトなどを格納しています。
  
# 参照

TODO:ROSパッケージ及びROS2パッケージのリポジトリを作成

# TODO

TODO: 非常停止時に減速して停止するように改良

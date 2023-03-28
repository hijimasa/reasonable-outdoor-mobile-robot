# 廉価版屋外向け移動ロボットプラットフォーム

このリポジトリは、つくばチャレンジや中之島チャレンジで個人参加しやすい廉価な屋外向け移動ロボットプラットフォームを実現するために立ち上げたものです。
できるだけ廉価になるように設計してみたのですが、材料費は20万円程度となってしまったので、個人が手軽に始められるかどうかはちょっと怪しいです。
私個人が実験的に作成したものなので、質問やissueなどにはできる限り回答しますが、利活用は各々の自己責任のもとで行ってください。
本リポジトリが屋外向け移動ロボット開発の一助となれば幸いです。

![ロボット外観](figs/reasonable_robot.png)

# 特徴

- Direct Drive Tech社のタイヤ付きモータを採用

  タイヤの組付けなどの作業を省略した上で、部品数の低減に貢献しています。

- 24V20Ah相当のバッテリー搭載

  12V20Ahの鉛蓄電池を2つ直列に搭載しました。常時フル稼働でモータを駆動させるわけではないのであれば、屋外実験に十分な容量があると考えています。

- 自作のモータコントローラにより非常停止やブレーキ解除を実現

  今回採用したモータには非常停止やブレーキの解除機能がなく、作業性や安全面で問題があったので、制御PCとは独立したArduinoで機能を実現しました。
  何らかの規格に則っているわけではないので、安全性などについては今後の課題となっています。

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

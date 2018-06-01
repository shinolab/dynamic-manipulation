導入手順

1.下記の依存ライブラリを予めインスト―ルしておく．
・Boost
・Eigen3
・OpenCV
・KinectSDK2

さらに，下記のソフトをインスト―ルしておく．
・CMake
・Git (※説明はgit Bashを前提)

2. Gitを使ってリモートリポジトリをcloneする．具体的には以下のコマンドを入力する．
2-1.フォルダを作成したい場所に移動する．
2-2. gitでgit clone http://gitlab.hapis.k.u-tokyo.ac.jp/furumoto/dynamic-levitation.git
設定によってはパスワードの入力を求められるので，gitlabに入るためのパスワードを入力する．
2-3 cd dynamic-levitation 
2-4 git submodule update --init --recursive

3.CMakeを使ってソリューションファイル・プロジェクトファイルを生成する．


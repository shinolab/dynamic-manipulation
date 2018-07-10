導入手順

1.下記の依存ライブラリを予めインスト―ルしておく．
・Boost(環境変数にBOOST_ROOTという名前でBoostのルートディレクトリのパスを追加しておくこと．)
・Eigen3
・OpenCV（使用する予定の構成(x64 or Win32)でビルドする）（※CまだMakeでビルドされたOpenCVでしか検証していません）
・KinectSDK2

さらに，下記のソフトをインスト―ルしておく．
・CMake
・Git (※説明はgit Bashを前提.)

2. Gitを使ってリモートリポジトリをcloneする．具体的には以下のコマンドを入力する．
2-1.フォルダを作成したい場所に移動する．
2-2. リモートリポジトリのファイルを複製する．
(git clone http://gitlab.hapis.k.u-tokyo.ac.jp/furumoto/dynamic-levitation.gitを入力する．)
設定によってはパスワードの入力を求められるので，gitlabに入るためのパスワードを入力する．
一度間違った資格情報（ユーザー名とパスワード）を入力すると，それがwindowsに記録されてしまい，gitlabにログインできなくなる．
間違えた場合は，windowsの資格情報マネージャーを開き，該当の資格情報を削除すること．
成功すると，dynamic-levitationというディレクトリが作成され，その中にソースファイルがダウンロードされる．
ただしこの段階は，dynamic-levitationが依存しているライブラリ(autd-softwareとads)はまだダウンロードされていない．
2-3 dynamic-levitationディレクトリに移動する(cd dynamic-levitationを入力する．) 
2-4 依存するソースファイルをダウンロードする．(git submodule update --init --recursiveと入力する．)

3.CMakeを使ってソリューションファイル・プロジェクトファイルを生成する．
生成に成功すると，DynamicLevitation.slnというソリューションファイルが生成される．

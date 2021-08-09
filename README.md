導入手順

1. 下記の依存ライブラリを予めインスト—ルしておく．
    - Boost(環境変数にBOOST_ROOTという名前でBoostのルートディレクトリのパスを追加しておくこと．)
    - Eigen3
    - OpenCV（使用する予定の構成(x64 / Win32, Debug / Release)でビルドする）（※CまだMakeでビルドされたOpenCVでしか検証していません）
    - KinectSDK
    
さらに，下記のソフトをインスト—ルしておく．
    - CMake
    - Git (※説明はgit Bashを前提.)

2. Gitを使ってリモートリポジトリをcloneする．具体的には以下のコマンドを入力する．
    1. フォルダを作成したい場所に移動する．（フォルダを作成する場所で右クリックし，"Git Bash Here"を押す）
    2. リモートリポジトリのファイルを複製する．
        (git clone http://gitlab.hapis.k.u-tokyo.ac.jp/furumoto/dynamic-levitation.git を入力する．)

        設定によってはパスワードの入力を求められるので，gitlabに入るためのパスワードを入力する．
        成功すると，dynamic-levitationというディレクトリが作成され，その中にソースファイルがダウンロードされる．
        ただしこの段階では，dynamic-manipulationが依存しているライブラリ(autd-softwareとads)はまだダウンロードされていない．
        ※間違った資格情報（ユーザー名とパスワード）を入力してしまうと，それがwindowsに記録されてしまいgitlabにログインできなくなる．
        間違えた場合は，windowsの資格情報マネージャーを軌道し，該当の資格情報を削除すること．
    3. dynamic-levitationディレクトリに移動する(cd dynamic-manipulationを入力する．) 
    4. 依存するソースファイルをダウンロードする．(git submodule update --init --recursiveと入力する．)成功すると，dynamic-manipulation/deps/autd3-software内に各種ファイルが生成される．

3. CMakeを使ってソリューションファイル・プロジェクトファイルを生成する．
場合によっては，
    - Eigen
    - OpenCV
のディレクトリパスを聞かれるので，適切なパスを入力する．
(OpenCVに関しては，CMakeを利用するとOpenCVディレクトリのどこかに"install"というディレクトリが生成されているはずなので，そのパスを指定する)

成功すると，DynamicLevitation.slnというソリューションファイルが生成される．

4. (サンプルプログラム：demo_manipulatorを動かす場合のみ)dataディレクトリ内のcam_stereo.ymlとblue_target_no_cover.pngを実行ファイルと同じディレクトリに移す．

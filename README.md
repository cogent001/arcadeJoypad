#IQkey_Remocon

설치 방법:

1. 원격 저장소에서 sync fork 버튼을 눌러 원래의 저장소와 코드의 내용을 일치시킵니다.

2. 로컬 저장소가 존재하지 않는 경우 local repository에 소스 코드를 내려 받습니다. 
   만약 로컬 저장소가 이미 존재한다면 아래와 같은 명령을 입력하여 변경된 코드를 로컬 저장소에 반옇합니다.

   git pull
   or
   git pull origin main 

3. ESP IDF 5.0 CMD 터머널을 열고 아래 명령문을 입력합니다.

    idf.py set-target esp32c3
    
4. sdkconfigRepo 폴더에서 sdkconfig 파일을 복사해서 상위 디렉토리에 복사합니다.

5. 아래와 같은 명령문을 입력합니다.
 
    idf.py menuconfig 

    메뉴 창이 열리면 partition Table > partionTable > Custom partition table CSV  선택후 's'키를 눌러 세이브 합니다
    이후 ESC키를 눌러 메뉴 창을 빠져 나옵니다

6. 아래와 같은 명령문을 입력하여 빌드합니다. 

    idf.py build

7. 빌드가 종료되면 디바이스를 연결한 후 아래 명령문을 입력하여 펌웨어를 업로드 합니다.

    idf.py -p COMx flash

    
    
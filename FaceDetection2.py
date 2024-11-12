from djitellopy import tello
from time import sleep
import cv2
from cvzone.PoseModule import PoseDetector

#無人機初始化,開始時升高到一定的程度
mytello = tello.Tello()
mytello.connect() #wifi連線
mytello.streamon() #開啓視訊鏡頭
mytello.takeoff() #起飛
print("電池格數剩: {}%".format(mytello.get_battery()))
mytello.send_rc_control(0, 0, 25, 0) #上升至一定的高度
sleep(2)

h = 600
w = 400
udRange = [150, 300]  #上下空間規範
lfRange = [240, 450]  #左右空間規範

detector = PoseDetector()
######追蹤臉部特徵########
def findFace(img):
    faceCascade = cv2.CascadeClassifier(r"C:\Users\14620\DJI-Tello-Drone-ComputerVision\haarcascade_frontalface_default.xml")##修改成自己的文件位置##
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(imgGray, 1.1, 8)
    myFaceList = []
    myFaceListArea = []
    #####畫出框架########
    for (x, y, w, h) in faces:
        cv2.rectangle(img, (x,y), (x+w, y+h), (0, 0, 255), 2)
        cx = x + w // 2
        cy = y + h // 2
        area = w * h
        myFaceList.append([cx, cy])
        myFaceListArea.append(area)
        cv2.circle(img, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
    #找出最大值,預防有多張人臉時不誤判
    if len(myFaceListArea) != 0:
        i = myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceList[i], myFaceListArea[i]]
    else:
        return img, [[0, 0], 0]
#########臉部偵測/使無人機上下左右跟隨##########
def trackFace(info):
    ud = 0 #上下
    yaw = 0 #左右
    area = info[1]
    x, y = info[0]
    if y < udRange[0] and y > udRange[1]:
        ud = 0
    elif y > udRange[1]:
        ud = -40
    elif y < udRange[0]:
        ud = 40
    if x < lfRange[0] and x > lfRange[1]:
        yaw = 0
    elif x > lfRange[1]:
        yaw = 40
    elif x < lfRange[0]:
        yaw = -40
    if y == 0:
        ud = 0
    if x == 0:
        yaw = 0
    mytello.send_rc_control(0, 0, ud, 0)
    mytello.send_rc_control(0, 0, 0, yaw)
    print(ud)
    print(yaw)

#####偵測身姿##############
savelr = []
fbsave = []
def poseDetector():
    lr = 0
    fb = 0
    lrList = []
    fbList = []
    lmList, bboxInfo = detector.findPosition(img, draw=False, bboxWithHands=True)
    if len(lmList) != 0:
        ###########左/右###############
        if lmList[14][2] < 230:
            lrList.append([lmList[14][1], lmList[16][1]])
            if lmList[16][1] > lmList[14][1]:
                lr = 30
                print("left")
            elif lmList[16][1] < lmList[14][1]:
                lr = -30
                print("right")
            mytello.send_rc_control(lr, 0, 0, 0)
        else:
            lrList.append([0, 0])
        ################前/後###############
        if lmList[13][2] < 230:
            fbList.append([lmList[13][2], lmList[15][2]])
            if lmList[13][1] < lmList[15][1]:
                fb = 30
                print("forward")
            elif lmList[13][2] > lmList[15][2]:
                fb = -30
                print("backup")
            mytello.send_rc_control(0, fb, 0, 0)
        else:
            fbList.append([0, 0])
        print("udList", fbList)
        print("lrList", lrList)

# cap = cv2.VideoCapture(0)
while True:
    # success, img = cap.read()
    img = mytello.get_frame_read().frame
    img = cv2.resize(img, (h, w))
    img = detector.findPose(img)
    poseDetector()
    img, info = findFace(img)
    trackFace(info)
    # print(info)

    cv2.imshow("img", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        mytello.land()
        break


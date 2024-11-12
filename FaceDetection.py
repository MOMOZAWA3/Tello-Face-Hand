from cvzone.HandTrackingModule import HandDetector
import cv2
from djitellopy import tello
from time import sleep

# 初始化Tello无人机
mytello = tello.Tello()
mytello.connect()
mytello.streamon()  # 仅开启摄像头流，暂不让无人机起飞
print("电池电量: {}".format(mytello.get_battery()))

# 初始化HandDetector
detector = HandDetector(detectionCon=0.5, maxHands=1)

# 飞行状态标志
flying = False

while True:
    # 获取图像帧
    img = mytello.get_frame_read().frame
    if img is None:
        print("未获取到图像帧")
        continue

    img = cv2.resize(img, (600, 400))
    
    # 检测手势并获取手部信息
    hands, img = detector.findHands(img)  # 使用 findHands 方法来检测手部信息
    
    if hands:
        hand = hands[0]  # 获取第一只手的信息
        lmList = hand.get("lmList", [])  # 获取手的关键点
        bbox = hand.get("bbox", [])  # 获取手的边界框

        if lmList:
            fingers = detector.fingersUp(hand)  # 检测手指伸展情况
            totalFingers = fingers.count(1)
            
            # 显示手指数量
            print(f"检测到的手指数: {totalFingers}")
            cv2.putText(img, f'Fingers:{totalFingers}', (bbox[0] + 200, bbox[1] - 30),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0, 255, 0), 2)
            
            # 如果检测到5根手指，并且无人机没有在飞行，触发起飞
            if totalFingers == 5 and not flying:
                print("检测到5根手指，准备起飞...")
                mytello.takeoff()
                flying = True
                sleep(2)  # 确保起飞动作执行

            # 如果无人机已经在飞行，检测其他手势动作
            if flying:
                if totalFingers == 1:
                    mytello.send_rc_control(0, 20, 0, 0)  # 向前飞
                elif totalFingers == 2:
                    mytello.send_rc_control(0, -20, 0, 0)  # 向后飞
                elif totalFingers == 3:
                    mytello.send_rc_control(20, 0, 0, 0)  # 向右飞
                elif totalFingers == 4:
                    mytello.send_rc_control(-20, 0, 0, 0)  # 向左飞
                elif totalFingers == 0:  # 停止飞行手势
                    print("检测到停止手势，正在降落...")
                    mytello.land()
                    flying = False
                else:
                    mytello.send_rc_control(0, 0, 0, 0)  # 停止
    # 显示图像
    cv2.imshow("img", img)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        if flying:
            print("正在降落...")
            mytello.land()
            flying = False
        break

cv2.destroyAllWindows()


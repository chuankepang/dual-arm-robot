import cv2
import mediapipe as mp


class HandDetector():
    '''
    手势识别类
    '''
    def __init__(self, mode=False, max_hands=2, modelC=1, detection_con=0.5, track_con=0.5):
        '''
        初始化
        :param mode: 是否静态图片，默认为False
        :param max_hands: 最多几只手，默认为2只
        :param detection_con: 最小检测信度值，默认为0.5
        :param track_con: 最小跟踪信度值，默认为0.5
        '''
        self.mode = mode
        self.max_hands = max_hands
        self.modelC = modelC
        self.detection_con = detection_con
        self.track_con = track_con

        self.hands = mp.solutions.hands.Hands(self.mode, self.max_hands,self.modelC, self.detection_con, self.track_con)

    def find_hands(self, img, draw=True):
        '''
        检测手势
        :param img: 视频帧图片
        :param draw: 是否画出手势中的节点和连接图
        :return: 处理过的视频帧图片
        '''
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # 处理图片，检测是否有手势，将数据存进self.results中
        self.results = self.hands.process(imgRGB)
        if draw:
            if self.results.multi_hand_landmarks:
                for handlms in self.results.multi_hand_landmarks:
                    mp.solutions.drawing_utils.draw_landmarks(img, handlms, mp.solutions.hands.HAND_CONNECTIONS)
        return img

    def find_positions(self, img, hand_no=0):
        '''
        获取手势数据
        :param img: 视频帧图片
        :param hand_no: 手编号（默认第1只手）
        :return: 手势数据列表，每个数据成员由id, x, y组成，代码这个手势位置编号以及在屏幕中的位置
        '''
        self.lmslist = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_no]
            for id, lm in enumerate(hand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                self.lmslist.append([id, cx, cy])

        return self.lmslist
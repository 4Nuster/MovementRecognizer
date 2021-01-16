import cv2
import math
from PyQt5 import QtCore, QtGui, QtWidgets

#displays the recognition result
def haha_printer_go_brr(state, output, font):

    if(state[0]==0 and state[1]==0 and state[2]==0 and state[3]==0 and state[4]==0 and state[5]==0  and state[6]==0):
        return -1

    maxx = max(state)

    if state[0]==maxx:
        cv2.putText(output, 'STANDING STILL', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 0
    elif state[1]==maxx:
        cv2.putText(output, 'WALKING ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 1
    elif state[2]==maxx:
        cv2.putText(output, 'RUNNING ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 2
    elif state[3]==maxx:
        cv2.putText(output, 'JUMPING ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 3
    elif state[4]==maxx:
        cv2.putText(output, 'SITTING ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 4
    elif state[5]==maxx:
        cv2.putText(output, 'LAYING DOWN ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 5
    elif state[6]==maxx:
        cv2.putText(output, 'WARNING: FALLING ', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return 6

def angle_calc(angle):
    if(angle > 90):
        something = abs(angle - 180)
    else:
        something = angle

    return something

class Results:
    def __init__(self, legs, ellipse, h, w, h_speed, v_speed, result):
        self.legs = legs
        self.ellipse = ellipse
        self.h = h
        self.w = w
        self.h_speed = h_speed
        self.v_speed = v_speed
        self.result = result

class Tests:
    def __init__(self, name, nframes, correctness):
        self.name = name
        self.nframes = nframes
        self.correctness = correctness 

#class Ui_MainWindow(object):
class Ui_MainWindow(QtWidgets.QWidget):
    #recognition using premade silhouettes
    def clicked(self):
        ellipse = (0, 0, 0)
        legs = 0
        h = 0
        w = 0
        h_speed = 0
        v_speed = 0
        printerResult = 0
        self.resultsList.clear()
        self.framecounter = 0
        #choosing dataset
        msg3 = ".PNG"
        
        if(self.S2testCombo.currentIndex()==0):
            msg1 = "../base mvt/Camera4/Sample1/GT-000"
            msg2 = 398
            msg4 = 463
            walking = 'WALKING'
        elif(self.S2testCombo.currentIndex()==1):
            msg1 = "../base mvt/daria_walk/"
            msg2 = 1
            msg4 = 85
            walking = 'WALKING'
        elif(self.S2testCombo.currentIndex()==2):
            msg1 = "../base mvt/Sample1/GT-001"
            msg2 = 100
            msg4 = 196
            walking = 'STANDING STILL'
        elif(self.S2testCombo.currentIndex()==3):
            msg1 = "../base mvt/Sample2/GT-000"
            msg2 = 565
            msg4 = 641
            walking = 'STANDING STILL'
        elif(self.S2testCombo.currentIndex()==4):
            msg1 = "../base mvt/Sample3/GT-000"
            msg2 = 463
            msg4 = 521
            walking = 'WALKING'
        
        #variables used in recognition
        #output_decision = 1
        spotted = False
        previous = None
        #sitting = False
        first_stand = False
        fall_warning_saver = False
        font = cv2.FONT_HERSHEY_COMPLEX

        #recognition frame by frame
        while ((msg2 < msg4) and not self.stop):
            legs = 0
            #acquiring next frame and processing it
            msg = msg1 + str(msg2) + msg3
            img = cv2.imread(msg)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            state = [0,0,0,0,0,0,0]

            #threshold_delta = cv2.threshold(gray, 29, 255, cv2.THRESH_BINARY)[1]
            #threshold_delta = cv2.dilate(threshold_delta, None, iterations= 0)

            #contours extraction
            contours, hierarchy = cv2.findContours(gray.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            output = img
            i1 = img.copy()

            if (len(contours) > 0):
                #contour extraction
                contour = max(contours, key= lambda heh: cv2.contourArea(heh))

                #calculating bounding rectangle and center point
                (x, y, w, h) = cv2.boundingRect(contour)
                center = (int((x*2+w)/2), int((y*2+h)/2))

                #approximating the contour
                epsilon = 0.0005 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                #calculating the convex hull and convexity defects
                hull = cv2.convexHull(approx, returnPoints=False)
                defects = cv2.convexityDefects(approx, hull)
        
                #variables used to extract lowest convexity defect
                #l=0
                #closest_start= None
                #closest_end = None
                printed_circle = False

                #going through all defects
                if(defects is not None):
                    for i in range(defects.shape[0]):
                        #extracting defect triangle points
                        s, e, f, d = defects[i,0]
                        start = tuple(approx[s][0])
                        end = tuple(approx[e][0])
                        far = tuple(approx[f][0])

                        #calculating defect triangle sides
                        a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                        b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                        c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                        abc = (a + b + c) / 2

                        #calculating distance and angle of the far point
                        d = (2 * math.sqrt(abc * (abc - a) * (abc - b) * (abc - c))) / a
                        angle = math.acos((b ** 2 + c ** 2 - a ** 2)/(2 * b * c)) * 57

                        #using angle for recognition

                        #if angle <= 90 and d>30:
                        if angle <= 90 and d > 20:
                            #l += 1
                            #if(far[1] > center[1]):
                            if(far[1] > center[1] and start[1] > center[1] and end[1] > center[1] and not printed_circle):
                                #visualizing the angle and its value
                                #if(self.angleCheckBox.isChecked()):
                                cv2.putText(output, 'Angle= '+str(int(angle)), (500, 490), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                legs = angle
                                #cv2.putText(output, 'Dist='+str(int(d)), (0, 130), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                #print('angle: ', int(angle))
                                cv2.circle(output, far, 3, (0, 0, 255), -1)
                                cv2.line(output, start, end, (0, 255, 0), 2)

                                cv2.circle(i1, far, 3, (0, 0, 255), -1)
                                cv2.line(i1, start, end, (0, 255, 0), 2)

                                #ignore the remaining defects
                                printed_circle = True
                                
                                #if (abs(start[1]-end[1])>25):
                                #    cv2.putText(output, 'KICK', (500, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                                if(d >= 50 and abs(start[1]-end[1]) < 25):
                                    #if(angle>=50):
                                    if(angle >= self.S3legs):
                                        #running motion detected using angle
                                        state[2] += 20
                                    else:
                                        #walking motion detected using angle
                                        state[1] += 20

                
                        #cv2.drawContours(output, hull, -1, (0, 255, 0))


                #resetting the variables once the silhouette is no longer detected
                if not spotted:
                    #spot_zone = (center[0]-50, center[1]-50, center[0]+50, center[1]+50)
                    spot_zone = center
                    previous = center
                    stand_o_meter = 0
                    spotted = True
                    continue
                
                #visualizing the center and the last-standing-spot rectangle
                #if(self.centerCheckBox.isChecked()):
                cv2.circle(output, center, 3,(0, 0, 0), -1)
                cv2.rectangle(output, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)
                cv2.circle(i1, center, 3,(0, 0, 0), -1)
                #cv2.rectangle(i1, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)

                #ellipse = cv2.fitEllipse(contour)
                #cv2.ellipse(output, ellipse, (255, 0, 0), 2)

                #visualizing the silhouette's bounding rectangle / bounding ellipse
                #cv2.rectangle(output, (x, y), (x+w, y+h), (255, 0, 0), 2)
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(output, ellipse, (0, 149, 255), 2)
                cv2.ellipse(i1, ellipse, (0, 149, 255), 2)
                cv2.putText(output, 'Ellipse angle= '+str(int(angle_calc(ellipse[2]))), (410, 460), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                #calculating the center's horizontal and vertical velocity
                h_speed = abs(center[0] - previous[0])
                v_speed = center[1] - previous[1]

                #visualizing the center's horizontal and vertical velocity
                cv2.putText(output, 'Vx= '+str(int(h_speed)), (500, 520), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(output, 'Vy= '+str(int(v_speed)), (500, 550), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                #using the bounding ellipse's angle and the bounding rectangle's dimensions and the vertical velocity to detect Falling/Laying Down/Sitting motions
                if(angle_calc(ellipse[2]) >= self.S3ellipse and angle_calc(ellipse[2]) < 80):
                    #Falling motion detected
                    state[6] = 50
                elif((w - ((3*h)/2)) > 0):
                    #if(v_speed > 3):
                    if(v_speed > 4):
                        #cv2.putText(output, 'WARNING: FALLING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        #Falling motion detected
                        state[6] = 50
                        fall_warning_saver = True
                    else:
                        #cv2.putText(output, 'LAYING DOWN', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        if(fall_warning_saver):
                            #Falling motions detected
                            state[6] = 50
                            fall_warning_saver = False
                        else:
                            #Laying Down motions detected
                            state[5] = 50
                elif(center[1] > (spot_zone[1] + 30)):
                    #cv2.putText(output, 'SITTING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    #if(v_speed > 1):
                    if(v_speed > 4):
                        #Falling motion detected
                        state[6] = 50
                        fall_warning_saver = True
                    else:
                        if(fall_warning_saver):
                            #Falling motion detected
                            state[6] = 50
                            #print('fall saved')
                            fall_warning_saver = False
                        else:
                            #Sitting motion detected
                            state[4] = 50
                    #print('SITTING: Vy= ',v_speed)
                #elif(abs(v_speed) > 5):
                    #cv2.putText(output, 'JUMPING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                #    state[3] = 50
                else:

                    #using horizontal velocity to detect Standing/Walking/Running motions
                    #if(h_speed < 7):
                    #if(h_speed < 5):
                    if(h_speed < self.S3walk):
                        stand_o_meter += 1
                        #cv2.putText(output, walking, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        if(walking=='WALKING'):
                            #Walking motion detected (just started walking/about to stop walking)
                            state[1] = 5
                        else:
                            #Standing motion detected
                            state[0] = 30
                    else:
                        stand_o_meter = 0
            
                        #if(h_speed <20):
                        #if((h_speed/(cv2.contourArea(contour)/10000)) < 15):
                        if((h_speed/(cv2.contourArea(contour)/10000)) < self.S3run):
                            #cv2.putText(output, 'WALKING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            #Walking motion detected
                            state[1] += 10
                            walking = 'WALKING'
                        else:
                            #cv2.putText(output, 'RUNNING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            #Running motion detected
                            state[2] += 10

        
                    if(stand_o_meter == 6):
                        #cv2.putText(output, 'STANDING STILL', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        #Standing motion detected
                        state[0] = 30
                        walking = 'STANDING STILL'
                        #refreshing the last-standing-spot
                        if first_stand:
                            spot_zone = (center[0], spot_zone[1])
                        else:
                            spot_zone = center
                            first_stand = True
                            fall_warning_saver = False

                        stand_o_meter = 0

                #saving the current frame's center's credentials to compare with the next frame (calculate velocity)
                previous = center

            else:
                #resetting variables once the silhouette is no longer detected
                spotted = False
                previous = None
                fall_warning_saver = False
                stand_o_meter = 0

            #visualizing the result
            printerResult = haha_printer_go_brr(state, output, font)
            printerResult += 1
            #cv2.imshow('TESTING', output)
            _translate = QtCore.QCoreApplication.translate
            self.S4display.setText(_translate("MainWindow", ""))
            #self.label.setPixmap(QtGui.QPixmap(output))
            height, width, channel = output.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(output.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
            self.S4display.setPixmap(QtGui.QPixmap(qImg))
            self.resultsList.append(Results(legs, angle_calc(ellipse[2]), h, w, h_speed, v_speed, printerResult))
            self.framecounter += 1
            cv2.imwrite("temp/"+str(self.framecounter)+"-1.png", i1)
            k = cv2.waitKey(150)
            if k == 27:
                break
    

            msg2 += 1

        #resetting the variables once there are no more frames to process
        _translate = _translate = QtCore.QCoreApplication.translate
        self.S4display.setText(_translate("MainWindow", "No input"))
        self.stop = False

    #real-time recognition
    def clicked2(self):
        legs = 0
        ellipse = (0, 0, 0)
        h = 0
        w = 0
        h_speed = 0
        v_speed = 0
        printerResult = 0
        self.resultsList.clear()
        self.framecounter = 0
        #variables used in recognition
        msgbox2error = 0
        if(self.S2webcamRadio.isChecked()):
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        elif(self.S2videoRadio.isChecked()):
            cap = cv2.VideoCapture(self.S2status.text())
        #output_decision = 1
        spotted = False
        previous = None
        walking = 'WALKING'
        #sitting = False
        first_stand = False
        self.first_img = None
        self.current = None
        font = cv2.FONT_HERSHEY_COMPLEX
        #kernel = np.ones((3,3), np.uint8)

        #recognition frame by frame
        while not self.stop:
            #acquiring the next frame and processing it
            #msg = Path(msg1 + str(msg2) + msg3)
            #msg = msg1 + str(msg2) + msg3
            #img = cv2.imread(msg)
            legs = 0

            ret, img = cap.read()

            img = cv2.flip(img, 1)
            #grayscaling
            try:
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            except:
                if(self.S2webcamRadio.isChecked()):
                    msgbox2 = QtWidgets.QMessageBox()
                    msgbox2.setWindowTitle("Error")
                    msgbox2.setText("No webcam detected")
                    msgbox2.setInformativeText("Please plug in a webcam and make sure it's functional to proceed")
                    msgbox2.setIcon(QtWidgets.QMessageBox.Warning)

                    x = msgbox2.exec_() 
                    msgbox2error = 1
                    break
                elif(self.S2videoRadio.isChecked()):
                    msgbox2error = 2
                    break

            #gaussian blur
            #gray = cv2.GaussianBlur(gray, (21, 21), 0)
            i1 = img.copy()
            gray = cv2.GaussianBlur(gray, (int(self.S3gb), int(self.S3gb)), 0)
            i2 = gray.copy()
            i2 = cv2.cvtColor(i2, cv2.COLOR_GRAY2BGR)
            state = [0,0,0,0,0,0,0]

            if self.first_img is None:
                self.first_img = gray
                continue

            self.current = gray
            
            #background subtraction
            
            delta_frame = cv2.absdiff(self.first_img, gray)
            i3 = delta_frame.copy()
            i3 = cv2.cvtColor(i3, cv2.COLOR_GRAY2BGR)

            #binarizing the image
            #threshold_delta = cv2.threshold(delta_frame, 29, 255, cv2.THRESH_BINARY)[1]
            threshold_delta = cv2.threshold(delta_frame, self.S3bi, 255, cv2.THRESH_BINARY)[1]
            #threshold_delta = cv2.dilate(threshold_delta, None, iterations= 0)
            threshold_delta = cv2.dilate(threshold_delta, None, iterations= self.S3dilate)

            i4 = threshold_delta.copy()
            i4 = cv2.cvtColor(i4, cv2.COLOR_GRAY2BGR)

            #contours extraction
            contours, hierarchy = cv2.findContours(threshold_delta.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            output = img

            #choosing the output
            if(self.S4outputCombo.currentIndex()==0):
                output = img
            elif(self.S4outputCombo.currentIndex()==1):
                output = gray
                output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            elif(self.S4outputCombo.currentIndex()==2):
                output = delta_frame
                output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
            elif(self.S4outputCombo.currentIndex()==3):
                output = threshold_delta
                output = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)

            if (len(contours) > 0):
                #contours extraction
                contour = max(contours, key= lambda heh: cv2.contourArea(heh))

                #calculating bounding rectangle and center point
                (x, y, w, h) = cv2.boundingRect(contour)
                center = (int((x*2+w)/2), int((y*2+h)/2))

                #approximating the contour
                epsilon = 0.0005 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                #calculating the convex hull and convexity defects
                hull = cv2.convexHull(approx, returnPoints=False)
                defects = cv2.convexityDefects(approx, hull)
        
                #variables used to extract lowest convexity defect
                #l=0
                #closest_start= None
                #closest_end = None
                printed_circle = False

                #going through all defects
                if(defects is not None):
                    for i in range(defects.shape[0]):
                        #extracting defect triangle points
                        s, e, f, d = defects[i,0]
                        start = tuple(approx[s][0])
                        end = tuple(approx[e][0])
                        far = tuple(approx[f][0])

                        #calculating defect triangle sides
                        a = math.sqrt((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2)
                        b = math.sqrt((far[0] - start[0]) ** 2 + (far[1] - start[1]) ** 2)
                        c = math.sqrt((end[0] - far[0]) ** 2 + (end[1] - far[1]) ** 2)
                        abc = (a+b+c)/2

                        #calculating distance and angle of the far point
                        d = (2 * math.sqrt(abc * (abc - a) * (abc - b) * (abc - c))) / a
                        angle = math.acos((b ** 2 + c ** 2 - a ** 2)/(2 * b * c)) * 57

                        #using angle for recognition

                        #if angle <= 90 and d>30:
                        if angle <=90 and d>20:
                            #l += 1
                            #if(far[1] > center[1]):
                            if(far[1] > center[1] and start[1] > center[1] and end[1] > center[1] and not printed_circle):
                                #visualizing the angle and its value
                                #if(self.angleCheckBox.isChecked()):
                                cv2.putText(output, 'Angle= '+str(int(angle)), (500, 490), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                legs = angle
                                #cv2.putText(output, 'Dist='+str(int(d)), (0, 130), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                                #print('angle: ', int(angle))
                                cv2.circle(output, far, 3, (0, 0, 255), -1)
                                cv2.line(output, start, end, (0, 255, 0), 2)
                                
                                cv2.circle(i1, far, 3, (0, 0, 255), -1)
                                cv2.line(i1, start, end, (0, 255, 0), 2)
                                cv2.circle(i2, far, 3, (0, 0, 255), -1)
                                cv2.line(i2, start, end, (0, 255, 0), 2)
                                cv2.circle(i3, far, 3, (0, 0, 255), -1)
                                cv2.line(i3, start, end, (0, 255, 0), 2)
                                cv2.circle(i4, far, 3, (0, 0, 255), -1)
                                cv2.line(i4, start, end, (0, 255, 0), 2)

                                #ignore the remaining defects
                                printed_circle = True

                                if(d>=50):
                                    if(angle>=self.S3legs and abs(start[1]-end[1])<25):
                                        #running motion detected using angle
                                        state[2] += 10
                                    else:
                                        #walking motion detected using angle
                                        state[1] += 10

                
                        #cv2.drawContours(output, hull, -1, (0, 255, 0))

                #resetting the variables once the silhouette is no longer detected
                if not spotted:
                    #spot_zone = (center[0]-50, center[1]-50, center[0]+50, center[1]+50)
                    spot_zone = center
                    previous = center
                    stand_o_meter = 0
                    spotted = True
                    continue

                #visualizing the center and the last-standing-spot rectangle
                #if(self.centerCheckBox.isChecked()):
                cv2.circle(output, center, 3,(0, 0, 0), -1)
                cv2.rectangle(output, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)
                
                cv2.circle(i1, center, 3,(0, 0, 0), -1)
                cv2.rectangle(i1, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)
                cv2.circle(i2, center, 3,(0, 0, 0), -1)
                cv2.rectangle(i2, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)
                cv2.circle(i3, center, 3,(0, 0, 0), -1)
                cv2.rectangle(i3, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)
                cv2.circle(i4, center, 3,(0, 0, 0), -1)
                cv2.rectangle(i4, (spot_zone[0]-30, spot_zone[1]-30), (spot_zone[0]+30, spot_zone[1]+30), (120, 0, 180), 2)

                #visualizing the silhouette's bounding rectangle / bounding ellipse
                #cv2.rectangle(output, (x, y), (x+w, y+h), (255, 0, 0), 2)
                ellipse = cv2.fitEllipse(contour)
                cv2.ellipse(output, ellipse, (0, 149, 255), 2)
                cv2.putText(output, 'Ellipse angle= '+str(int(angle_calc(ellipse[2]))), (410, 460), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.ellipse(i1, ellipse, (0, 149, 255), 2)
                cv2.ellipse(i2, ellipse, (0, 149, 255), 2)
                cv2.ellipse(i3, ellipse, (0, 149, 255), 2)
                cv2.ellipse(i4, ellipse, (0, 149, 255), 2)

                #calculating the center's horizontal and vertical velocity
                h_speed = abs(center[0] - previous[0])
                v_speed = center[1] - previous[1]

                #visualizing the center's horizontal and vertical velocity
                cv2.putText(output, 'Vx= '+str(int(h_speed)), (500, 520), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(output, 'Vy= '+str(int(v_speed)), (500, 550), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                #using the bounding ellipse's angle and the rectangle's dimensions and the vertical velocity to detect Falling/Laying Down/Sitting motions
                if(angle_calc(ellipse[2]) >= self.S3ellipse and angle_calc(ellipse[2]) < 80):
                    #Falling motion detected
                    state[6] = 50
                elif((w - ((3*h)/2)) > 0):
                    #if(v_speed > 3):
                    if(v_speed > 4):
                        #cv2.putText(output, 'WARNING: FALLING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        #Falling motion detected
                        state[6] = 50
                    else:
                        #cv2.putText(output, 'LAYING DOWN', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        #Laying Down motions detected
                        state[5] = 50
                elif(center[1] > (spot_zone[1] + 30)):
                    #cv2.putText(output, 'SITTING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                    if(v_speed > 4):
                        #Falling motion detected
                        state[6] = 50
                    else:
                        #Sitting motion detected
                        state[4] = 50
                #elif(abs(v_speed) > 5):
                    #cv2.putText(output, 'JUMPING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                #    state[3] = 50
                else:

                    #using horizontal velocity to detect Standing/Walking/Running motions
                    #if(h_speed < 7):
                    #if(h_speed < 5):
                    if(h_speed < self.S3walk):
                        stand_o_meter += 1
                        #cv2.putText(output, walking, (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        if(walking=='WALKING'):
                            #Walking motion detected (just started walking/about to stop walking)
                            state[1] = 5
                        else:
                            #Standing motion detected
                            state[0] = 30
                    else:
                        stand_o_meter = 0
            
                        #if(h_speed <20):
                        #if((h_speed/(cv2.contourArea(contour)/10000)) < 15):
                        if((h_speed/(cv2.contourArea(contour)/10000)) < self.S3run):
                            #cv2.putText(output, 'WALKING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            #Walking motion detected
                            state[1] += 20
                            walking = 'WALKING'
                        else:
                            #cv2.putText(output, 'RUNNING', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                            #Running motion detected
                            state[2] += 20

        
                    if(stand_o_meter == 6):
                        #cv2.putText(output, 'STANDING STILL', (0, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                        #Standing motion detected
                        state[0] = 30
                        walking = 'STANDING STILL'
                        #refreshing the last-standing-spot
                        if first_stand:
                            spot_zone = (center[0], spot_zone[1])
                        else:
                            spot_zone = center
                            first_stand = True

                        stand_o_meter = 0
        
                #saving the current frame's center's credentials to compare with the next frame (calculate velocity)
                previous = center

            else:
                #resetting variables once the silhouette is no longer detected
                spotted = False
                previous = None
                stand_o_meter = 0

            #visualizing the result
            printerResult = haha_printer_go_brr(state, output, font)
            printerResult += 1
            #cv2.imshow('TESTING', output)
            _translate = QtCore.QCoreApplication.translate
            self.S4display.setText(_translate("MainWindow", ""))
            #self.label.setPixmap(QtGui.QPixmap(output))
            height, width, channel = output.shape
            bytesPerLine = 3 * width
            qImg = QtGui.QImage(output.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
            self.S4display.setPixmap(QtGui.QPixmap(qImg))
            self.resultsList.append(Results(legs, angle_calc(ellipse[2]), h, w, h_speed, v_speed, printerResult))
            self.framecounter += 1
            cv2.imwrite("temp/"+str(self.framecounter)+"-1.png", i1)
            cv2.imwrite("temp/"+str(self.framecounter)+"-2.png", i2)
            cv2.imwrite("temp/"+str(self.framecounter)+"-3.png", i3)
            cv2.imwrite("temp/"+str(self.framecounter)+"-4.png", i4)

            #print(h_speed, '/', cv2.contourArea(contour)/100, '=', h_speed/(cv2.contourArea(contour)/100))
            if(self.S2webcamRadio.isChecked()):
                k = cv2.waitKey(30)
            elif(self.S2videoRadio.isChecked()):
                k = cv2.waitKey(60)
            if k == 27:
                break
    

            #msg2 += 1

        #resetting the variables once the Stop button is pressed
        #cv2.destroyAllWindows()
        _translate = QtCore.QCoreApplication.translate
        self.S4display.setText(_translate("MainWindow", "No input"))
        cap.release
        if(msgbox2error == 1):
            self.S4back()
        elif(msgbox2error == 2):
            self.S4stop()

        self.stop = False
    
    resultsList = []
    testsList = []
    framecounter = 0

    def display(self, outputt):
        _translate = QtCore.QCoreApplication.translate
        #self.label.setPixmap(QtGui.QPixmap(outputt))
        height, width, channel = outputt.shape
        bytesPerLine = 3 * width
        qImg = QtGui.QImage(outputt.data, width, height, bytesPerLine, QtGui.QImage.Format_RGB888).rgbSwapped()
        return QtGui.QPixmap(qImg)

    def analysis(self):
        if(len(self.resultsList) != 0):
            try:
                something = self.resultsList[int(self.S5frame.value()-1)]
            except:
                print("expected frame: "+str(self.S5frame.value()-1)+"but got: "+str(len(self.resultsList)))

            self.S5cxLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Centroid's X Axis Difference = "+str(something.h_speed)))
            self.S5cyLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Centroid's Y Axis Difference = "+str(something.v_speed)))
            self.S5legsLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Angle Between Legs = "+str(something.legs)))
            self.S5ellipseLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Bounding Ellipse's Angle = "+str(something.ellipse)))
            self.S5wLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Bounding Rectangle's W = "+str(something.w)))
            self.S5hLabel.setText(QtCore.QCoreApplication.translate("MainWindow", "Bounding Rectangle's H = "+str(something.h)))
            #print(something.h / something.w)

            self.S5result.setText("")
            if(something.result == 0):
                self.S5result.setPixmap(QtGui.QPixmap("results/Nothing.png"))
            elif(something.result == 1):
                self.S5result.setPixmap(QtGui.QPixmap("results/Standing.png"))
            elif(something.result == 2):
                self.S5result.setPixmap(QtGui.QPixmap("results/Walking.png"))
            elif(something.result == 3):
                self.S5result.setPixmap(QtGui.QPixmap("results/Running.png"))
            elif(something.result == 4):
                self.S5result.setPixmap(QtGui.QPixmap("results/Nothing.png"))
            elif(something.result == 5):
                self.S5result.setPixmap(QtGui.QPixmap("results/Sitting.png"))
            elif(something.result == 6):
                self.S5result.setPixmap(QtGui.QPixmap("results/Laying.png"))
            elif(something.result == 7):
                self.S5result.setPixmap(QtGui.QPixmap("results/Falling.png"))

    def S1proceed(self):
        self.inputTab.setEnabled(True)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(1)
        self.S2webcamRadio.setChecked(True)
    
    def S2open(self):
        name, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open",".","Video files (*.mp4 *.avi)")
        self.S2videoRadio.setChecked(True)
        self.S2status.setText(QtCore.QCoreApplication.translate("MainWindow", name))
    
    def S2test(self):
        self.S2testRadio.setChecked(True)
        self.S2status.setText(QtCore.QCoreApplication.translate("MainWindow", "Chosen Test: "+self.S2testCombo.currentText()))
    
    def S2proceed(self):
        if((self.S2status.text() != "Chosen method: Video") and (self.S2status.text() != "")):
            self.inputTab.setEnabled(False)
            self.settingsTab.setEnabled(True)
            self.previewTab.setEnabled(False)
            self.detailedTab.setEnabled(False)
            self.finalTab.setEnabled(False)
            self.tabWidget.setCurrentIndex(2)
            self.S3groupBox.setEnabled(True)
            if(self.S2testRadio.isChecked()):
                self.S3groupBox.setEnabled(False)  
        else:
            self.video_error()
    
    def video_error(self):
        msgbox = QtWidgets.QMessageBox()
        msgbox.setWindowTitle("Error")
        msgbox.setText("Please open a video to proceed")
        msgbox.setIcon(QtWidgets.QMessageBox.Warning)

        x = msgbox.exec_() 

    def S2back(self):
        self.inputTab.setEnabled(True)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(0)
    
    def S3back(self):
        self.inputTab.setEnabled(True)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(1)

    def S3default(self):
        self.S3gbSpin.setProperty("value", 21)
        self.S3biSpin.setProperty("value", 29)
        self.S3dilateSpin.setProperty("value", 0)
        self.S3legsSpin.setProperty("value", 50)
        self.S3walkSpin.setProperty("value", 5)
        self.S3runSpin.setProperty("value", 15)
        self.S3ellipseSpin.setProperty("value", 18)

    def S3proceed(self):
        self.inputTab.setEnabled(False)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(True)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(3)
        self.S4bgButton.setEnabled(True)
        self.S4outputCombo.setEnabled(True)
        self.S4outputLabel.setEnabled(True)
        if(self.S2testRadio.isChecked()):
            self.S4bgButton.setEnabled(False)
            self.S4outputCombo.setEnabled(False)
            self.S4outputLabel.setEnabled(False)
        else:
            self.S4outputCombo.setCurrentIndex(self.S3outputCombo.currentIndex())
        
        self.save_thresh()
        if(self.S2webcamRadio.isChecked() or self.S2videoRadio.isChecked()):
            self.clicked2()
        elif(self.S2testRadio.isChecked()):
            self.clicked()
        if(self.tabWidget.currentIndex() == 3):
            self.S4stop()

    S3gb = 0
    S3bi = 0
    S3dilate = 0
    S3legs = 0
    S3walk = 0
    S3run = 0
    S3ellipse = 0

    def save_thresh(self):
        self.S3gb = self.S3gbSpin.value()
        if(self.S3gb % 2 == 0):
            self.S3gb += 1
            self.S3gbSpin.setProperty("value", self.S3gb)
        self.S3bi = self.S3biSpin.value()
        self.S3dilate = self.S3dilateSpin.value()
        self.S3legs = self.S3legsSpin.value()
        self.S3walk = self.S3walkSpin.value()
        self.S3run = self.S3runSpin.value()
        self.S3ellipse = self.S3ellipseSpin.value()

    stop = False
    first_img = None

    def S4stop(self):
        self.stop = True
        self.inputTab.setEnabled(False)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(True)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(4)
        self.S5outputCombo.setEnabled(True)
        self.S5outputLabel.setEnabled(True)
        self.S5outputCombo.setCurrentIndex(0)
        if(self.S2testRadio.isChecked()):
            self.S5outputCombo.setEnabled(False)
            self.S5outputLabel.setEnabled(False)

        self.S5display.setText("")
        self.S5display.setPixmap(self.display(cv2.imread("temp/1-1.png")))
        self.S5frame.display(1)
        self.analysis()
        self.S5nextButton.setEnabled(True)
        self.S5previousButton.setEnabled(False)
        self.S5firstButton.setEnabled(False)

    
    #def S4restart(self):
    #    self.stop = True
    #    self.S3proceed()
    
    def S4bg(self):
        self.first_img = self.current

    def S4back(self):
        self.stop = True
        self.inputTab.setEnabled(False)
        self.settingsTab.setEnabled(True)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)
        self.tabWidget.setCurrentIndex(2)

    def S5next(self):
        something = int(self.S5frame.value())
        if(something == 1):
            self.S5previousButton.setEnabled(True)
            self.S5firstButton.setEnabled(True)
        something += 1
        self.S5frame.display(something)
        self.S5display.setPixmap(self.display(cv2.imread("temp/"+str(something)+"-"+str(self.S5outputCombo.currentIndex()+1)+".png")))
        self.analysis()
        if(something >= self.framecounter):
            self.S5nextButton.setEnabled(False)
    
    def S5previous(self):
        something = int(self.S5frame.value())
        something -= 1
        self.S5frame.display(something)
        self.S5display.setPixmap(self.display(cv2.imread("temp/"+str(something)+"-"+str(self.S5outputCombo.currentIndex()+1)+".png")))
        self.analysis()
        if(something == 1):
            self.S5previousButton.setEnabled(False)
            self.S5firstButton.setEnabled(False)
        if(something < self.framecounter):
            self.S5nextButton.setEnabled(True)

    def S5first(self):
        self.S5frame.display(1)
        self.S5display.setPixmap(self.display(cv2.imread("temp/1-"+str(self.S5outputCombo.currentIndex()+1)+".png")))
        self.analysis()
        self.S5previousButton.setEnabled(False)
        self.S5firstButton.setEnabled(False)
        self.S5nextButton.setEnabled(True)

    def S5back2(self, i):
        if(i.text() == "&Yes"):
            self.stop = False
            self.S1proceed()
    
    def S5back(self):
        msgbox3 = QtWidgets.QMessageBox()
        msgbox3.setWindowTitle("Discard Test?")
        msgbox3.setText("Going back will discard the current test, continue?")
        msgbox3.setIcon(QtWidgets.QMessageBox.Question)
        msgbox3.setStandardButtons(QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No)
        msgbox3.setDefaultButton(QtWidgets.QMessageBox.No)
        msgbox3.buttonClicked.connect(self.S5back2)

        x = msgbox3.exec_()

    def S5switch(self):
        something = int(self.S5frame.value())
        self.S5display.setPixmap(self.display(cv2.imread("temp/"+str(something)+"-"+str(self.S5outputCombo.currentIndex()+1)+".png")))

    def S5proceed(self):
        self.stop = False
        self.inputTab.setEnabled(False)
        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(True)
        self.tabWidget.setCurrentIndex(5)

        self.S6precent.setText("...")
        self.S6correctGroup.setEnabled(True)
        self.S6proceedButton.setEnabled(False)
        self.S6correctLabel.setText("")
        self.dominantMove()

    def dominantMove(self):
        i = 0
        resultsCounter = [0, 0, 0, 0, 0, 0, 0, 0]
        resultsPercent = [0, 0, 0, 0, 0, 0, 0, 0]
        for something in self.resultsList:
            resultsCounter[something.result] += 1
        for something in resultsCounter:
            #print(str((something / self.framecounter)*100))
            resultsPercent[i] = (something / self.framecounter)*100
            i += 1
        if(resultsPercent[7] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Falling.png"))
        elif(resultsPercent[3] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Running.png"))
        elif(resultsPercent[2] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Walking.png"))
        elif(resultsPercent[6] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Laying.png"))
        elif(resultsPercent[5] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Sitting.png"))
        elif(resultsPercent[1] >= 10):
            self.S6result.setPixmap(QtGui.QPixmap("results/Standing.png"))
        else:
            self.S6result.setPixmap(QtGui.QPixmap("results/Nothing.png"))
        
        self.S6leftRTF.setHtml(QtCore.QCoreApplication.translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
        "p, li { white-space: pre-wrap; }\n"
        "</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\">All Recognized moves:</span></p>\n"
        "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:14pt; font-weight:600;\"><br /></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Standing ("+str(int(resultsPercent[1]))+"% | "+str(resultsCounter[1])+" out of "+str(self.framecounter)+" frames)</span></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Walking ("+str(int(resultsPercent[2]))+"% | "+str(resultsCounter[2])+" out of "+str(self.framecounter)+" frames)</span></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Running ("+str(int(resultsPercent[3]))+"% | "+str(resultsCounter[3])+" out of "+str(self.framecounter)+" frames)</span></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Sitting ("+str(int(resultsPercent[5]))+"% | "+str(resultsCounter[5])+" out of "+str(self.framecounter)+" frames)</span></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Falling ("+str(int(resultsPercent[7]))+"% | "+str(resultsCounter[7])+" out of "+str(self.framecounter)+" frames)</span></p>\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Laying ("+str(int(resultsPercent[6]))+"% | "+str(resultsCounter[6])+" out of "+str(self.framecounter)+" frames)</span></p></body></html>"))

    def S6yes(self):
        self.S6correctLabel.setText("Correct Recognition")
        self.S6correctGroup.setEnabled(False)
        self.calc_accuracy(True)
        self.S6proceedButton.setEnabled(True)
        

    def S6no(self):
        self.S6correctLabel.setText("Incorrect Recognition")
        self.S6correctGroup.setEnabled(False)
        self.calc_accuracy(False)
        self.S6proceedButton.setEnabled(True)
    
    def calc_accuracy(self, correctness):
        if(self.S2webcamRadio.isChecked()):
            name = "Webcam"
        elif(self.S2videoRadio.isChecked()):
            name = self.S2status.text()
        elif(self.S2testRadio.isChecked()):
            name = self.S2testCombo.currentText()

        self.testsList.append(Tests(name, self.framecounter, correctness))
        allTests = ""
        correctOnes = 0
        total = len(self.testsList)
        for something in self.testsList:
            if(something.correctness):
                allTests += "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- "+something.name+" ("+str(something.nframes)+" frames) [Correct]</span></p>\n"
            else:
                allTests += "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- "+something.name+" ("+str(something.nframes)+" frames) [Incorrect]</span></p>\n"

            if(something.correctness):
                correctOnes += 1

        self.S6precent.setText(str(int((correctOnes/total)*100)))
        self.S6tests.setText(str(correctOnes)+"/"+str(total))

        self.S6rightRTF.setHtml(QtCore.QCoreApplication.translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
        "p, li { white-space: pre-wrap; }\n"
        "</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\">All tests:</span></p>\n"
        "<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:14pt; font-weight:600;\"><br /></p>\n"+allTests+"</body></html>"))

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(806, 580)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.tabWidget = QtWidgets.QTabWidget(self.centralwidget)
        self.tabWidget.setGeometry(QtCore.QRect(0, 0, 811, 561))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.tabWidget.setFont(font)
        self.tabWidget.setObjectName("tabWidget")
        self.mainTab = QtWidgets.QWidget()
        self.mainTab.setObjectName("mainTab")
        self.S1exitButton = QtWidgets.QPushButton(self.mainTab)
        self.S1exitButton.setGeometry(QtCore.QRect(240, 30, 151, 81))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        font.setBold(False)
        font.setWeight(50)
        self.S1exitButton.setFont(font)
        self.S1exitButton.setObjectName("S1exitButton")
        self.S1mainLogo = QtWidgets.QLabel(self.mainTab)
        self.S1mainLogo.setGeometry(QtCore.QRect(30, 250, 741, 231))
        self.S1mainLogo.setText("")
        self.S1mainLogo.setPixmap(QtGui.QPixmap("icons/main-logo-5.png"))
        self.S1mainLogo.setScaledContents(True)
        self.S1mainLogo.setAlignment(QtCore.Qt.AlignCenter)
        self.S1mainLogo.setObjectName("S1mainLogo")
        self.S1usthbLogo = QtWidgets.QLabel(self.mainTab)
        self.S1usthbLogo.setGeometry(QtCore.QRect(460, 150, 81, 71))
        self.S1usthbLogo.setText("")
        self.S1usthbLogo.setPixmap(QtGui.QPixmap("icons/usthb.png"))
        self.S1usthbLogo.setScaledContents(True)
        self.S1usthbLogo.setObjectName("S1usthbLogo")
        self.S1feiLogo = QtWidgets.QLabel(self.mainTab)
        self.S1feiLogo.setGeometry(QtCore.QRect(360, 150, 81, 71))
        self.S1feiLogo.setText("")
        self.S1feiLogo.setPixmap(QtGui.QPixmap("icons/FEI.png"))
        self.S1feiLogo.setScaledContents(True)
        self.S1feiLogo.setObjectName("S1feiLogo")
        self.S1diLogo = QtWidgets.QLabel(self.mainTab)
        self.S1diLogo.setGeometry(QtCore.QRect(260, 150, 81, 71))
        self.S1diLogo.setText("")
        self.S1diLogo.setPixmap(QtGui.QPixmap("icons/DI.png"))
        self.S1diLogo.setScaledContents(True)
        self.S1diLogo.setObjectName("S1diLogo")
        self.S1proceedButton = QtWidgets.QPushButton(self.mainTab)
        self.S1proceedButton.setGeometry(QtCore.QRect(410, 30, 151, 81))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        self.S1proceedButton.setFont(font)
        self.S1proceedButton.setObjectName("S1proceedButton")
        self.tabWidget.addTab(self.mainTab, "")
        self.inputTab = QtWidgets.QWidget()
        self.inputTab.setObjectName("inputTab")
        self.S2webcamRadio = QtWidgets.QRadioButton(self.inputTab)
        self.S2webcamRadio.setGeometry(QtCore.QRect(210, 300, 82, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2webcamRadio.setFont(font)
        self.S2webcamRadio.setObjectName("S2webcamRadio")
        self.S2videoRadio = QtWidgets.QRadioButton(self.inputTab)
        self.S2videoRadio.setGeometry(QtCore.QRect(340, 300, 121, 20))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2videoRadio.setFont(font)
        self.S2videoRadio.setObjectName("S2videoRadio")
        self.S2testRadio = QtWidgets.QRadioButton(self.inputTab)
        self.S2testRadio.setGeometry(QtCore.QRect(516, 300, 101, 21))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2testRadio.setFont(font)
        self.S2testRadio.setObjectName("S2testRadio")
        self.S2logo = QtWidgets.QLabel(self.inputTab)
        self.S2logo.setGeometry(QtCore.QRect(140, 40, 521, 241))
        self.S2logo.setText("")
        self.S2logo.setPixmap(QtGui.QPixmap("icons/step-1.png"))
        self.S2logo.setScaledContents(True)
        self.S2logo.setAlignment(QtCore.Qt.AlignCenter)
        self.S2logo.setObjectName("S2logo")
        self.S2testCombo = QtWidgets.QComboBox(self.inputTab)
        self.S2testCombo.setGeometry(QtCore.QRect(510, 330, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2testCombo.setFont(font)
        self.S2testCombo.setObjectName("S2testCombo")
        self.S2testCombo.addItem("")
        self.S2testCombo.addItem("")
        self.S2testCombo.addItem("")
        self.S2testCombo.addItem("")
        self.S2testCombo.addItem("")
        self.S2openButton = QtWidgets.QPushButton(self.inputTab)
        self.S2openButton.setGeometry(QtCore.QRect(340, 330, 121, 31))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2openButton.setFont(font)
        self.S2openButton.setObjectName("S2openButton")
        self.S2status = QtWidgets.QLabel(self.inputTab)
        self.S2status.setGeometry(QtCore.QRect(170, 380, 461, 16))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S2status.setFont(font)
        self.S2status.setAlignment(QtCore.Qt.AlignCenter)
        self.S2status.setObjectName("S2status")
        self.S2backButton = QtWidgets.QPushButton(self.inputTab)
        self.S2backButton.setGeometry(QtCore.QRect(240, 420, 151, 81))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        font.setBold(False)
        font.setWeight(50)
        self.S2backButton.setFont(font)
        self.S2backButton.setObjectName("S2backButton")
        self.S2proceedButton = QtWidgets.QPushButton(self.inputTab)
        self.S2proceedButton.setGeometry(QtCore.QRect(410, 420, 151, 81))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        self.S2proceedButton.setFont(font)
        self.S2proceedButton.setObjectName("S2proceedButton")
        self.tabWidget.addTab(self.inputTab, "")
        self.settingsTab = QtWidgets.QWidget()
        self.settingsTab.setObjectName("settingsTab")
        self.S3logo = QtWidgets.QLabel(self.settingsTab)
        self.S3logo.setGeometry(QtCore.QRect(80, 20, 641, 191))
        self.S3logo.setText("")
        self.S3logo.setPixmap(QtGui.QPixmap("icons/step-2.png"))
        self.S3logo.setScaledContents(True)
        self.S3logo.setAlignment(QtCore.Qt.AlignCenter)
        self.S3logo.setObjectName("S3logo")
        self.S3groupBox = QtWidgets.QGroupBox(self.settingsTab)
        self.S3groupBox.setEnabled(True)
        self.S3groupBox.setGeometry(QtCore.QRect(150, 220, 241, 161))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S3groupBox.setFont(font)
        self.S3groupBox.setObjectName("S3groupBox")
        self.S3gbSpin = QtWidgets.QSpinBox(self.S3groupBox)
        self.S3gbSpin.setGeometry(QtCore.QRect(190, 30, 41, 22))
        self.S3gbSpin.setProperty("value", 21)
        self.S3gbSpin.setObjectName("S3gbSpin")
        self.S3gbLabel = QtWidgets.QLabel(self.S3groupBox)
        self.S3gbLabel.setGeometry(QtCore.QRect(10, 30, 131, 21))
        self.S3gbLabel.setObjectName("S3gbLabel")
        self.S3biSpin = QtWidgets.QSpinBox(self.S3groupBox)
        self.S3biSpin.setGeometry(QtCore.QRect(190, 60, 41, 22))
        self.S3biSpin.setProperty("value", 29)
        self.S3biSpin.setObjectName("S3biSpin")
        self.S3biLabel = QtWidgets.QLabel(self.S3groupBox)
        self.S3biLabel.setGeometry(QtCore.QRect(10, 60, 131, 21))
        self.S3biLabel.setObjectName("S3biLabel")
        self.S3dilateSpin = QtWidgets.QSpinBox(self.S3groupBox)
        self.S3dilateSpin.setGeometry(QtCore.QRect(190, 90, 41, 22))
        self.S3dilateSpin.setObjectName("S3dilateSpin")
        self.S3dilateLabel = QtWidgets.QLabel(self.S3groupBox)
        self.S3dilateLabel.setGeometry(QtCore.QRect(10, 90, 131, 21))
        self.S3dilateLabel.setObjectName("S3dilateLabel")
        self.S3outputCombo = QtWidgets.QComboBox(self.S3groupBox)
        self.S3outputCombo.setGeometry(QtCore.QRect(69, 120, 161, 22))
        self.S3outputCombo.setObjectName("S3outputCombo")
        self.S3outputCombo.addItem("")
        self.S3outputCombo.addItem("")
        self.S3outputCombo.addItem("")
        self.S3outputCombo.addItem("")
        self.S3outputLabel = QtWidgets.QLabel(self.S3groupBox)
        self.S3outputLabel.setGeometry(QtCore.QRect(10, 120, 51, 21))
        self.S3outputLabel.setObjectName("S3outputLabel")
        self.S3groupBox_2 = QtWidgets.QGroupBox(self.settingsTab)
        self.S3groupBox_2.setGeometry(QtCore.QRect(410, 220, 241, 161))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.S3groupBox_2.setFont(font)
        self.S3groupBox_2.setObjectName("S3groupBox_2")
        self.S3legsLabel = QtWidgets.QLabel(self.S3groupBox_2)
        self.S3legsLabel.setGeometry(QtCore.QRect(10, 30, 131, 21))
        self.S3legsLabel.setObjectName("S3legsLabel")
        self.S3legsSpin = QtWidgets.QSpinBox(self.S3groupBox_2)
        self.S3legsSpin.setGeometry(QtCore.QRect(190, 30, 41, 22))
        self.S3legsSpin.setMaximum(89)
        self.S3legsSpin.setProperty("value", 50)
        self.S3legsSpin.setObjectName("S3legsSpin")
        self.S3walkSpin = QtWidgets.QSpinBox(self.S3groupBox_2)
        self.S3walkSpin.setGeometry(QtCore.QRect(190, 60, 41, 22))
        self.S3walkSpin.setMinimum(1)
        self.S3walkSpin.setMaximum(14)
        self.S3walkSpin.setProperty("value", 5)
        self.S3walkSpin.setObjectName("S3walkSpin")
        self.S3walkLabel = QtWidgets.QLabel(self.S3groupBox_2)
        self.S3walkLabel.setGeometry(QtCore.QRect(10, 60, 171, 21))
        self.S3walkLabel.setObjectName("S3walkLabel")
        self.S3runSpin = QtWidgets.QSpinBox(self.S3groupBox_2)
        self.S3runSpin.setGeometry(QtCore.QRect(190, 90, 41, 22))
        self.S3runSpin.setMinimum(6)
        self.S3runSpin.setProperty("value", 15)
        self.S3runSpin.setObjectName("S3runSpin")
        self.S3runLabel = QtWidgets.QLabel(self.S3groupBox_2)
        self.S3runLabel.setGeometry(QtCore.QRect(10, 90, 171, 21))
        self.S3runLabel.setObjectName("S3runLabel")
        self.S3ellipseSpin = QtWidgets.QSpinBox(self.S3groupBox_2)
        self.S3ellipseSpin.setGeometry(QtCore.QRect(190, 120, 41, 22))
        self.S3ellipseSpin.setMinimum(1)
        self.S3ellipseSpin.setMaximum(79)
        self.S3ellipseSpin.setProperty("value", 18)
        self.S3ellipseSpin.setObjectName("S3ellipseSpin")
        self.S3ellipseLabel = QtWidgets.QLabel(self.S3groupBox_2)
        self.S3ellipseLabel.setGeometry(QtCore.QRect(10, 120, 141, 21))
        self.S3ellipseLabel.setObjectName("S3ellipseLabel")
        self.S3defaultButton = QtWidgets.QPushButton(self.settingsTab)
        self.S3defaultButton.setGeometry(QtCore.QRect(190, 390, 421, 41))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(12)
        self.S3defaultButton.setFont(font)
        self.S3defaultButton.setObjectName("S3defaultButton")
        self.S3backButton = QtWidgets.QPushButton(self.settingsTab)
        self.S3backButton.setGeometry(QtCore.QRect(240, 440, 151, 61))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        font.setBold(False)
        font.setWeight(50)
        self.S3backButton.setFont(font)
        self.S3backButton.setObjectName("S3backButton")
        self.S3proceedButton = QtWidgets.QPushButton(self.settingsTab)
        self.S3proceedButton.setGeometry(QtCore.QRect(410, 440, 151, 61))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        self.S3proceedButton.setFont(font)
        self.S3proceedButton.setObjectName("S3proceedButton")
        self.tabWidget.addTab(self.settingsTab, "")
        self.previewTab = QtWidgets.QWidget()
        self.previewTab.setObjectName("previewTab")
        self.S4display = QtWidgets.QLabel(self.previewTab)
        self.S4display.setGeometry(QtCore.QRect(125, 10, 541, 421))
        self.S4display.setAlignment(QtCore.Qt.AlignCenter)
        self.S4display.setObjectName("S4display")
        self.S4display.setScaledContents(True)
        self.S4stopButton = QtWidgets.QPushButton(self.previewTab)
        self.S4stopButton.setGeometry(QtCore.QRect(350, 440, 91, 71))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(18)
        self.S4stopButton.setFont(font)
        self.S4stopButton.setObjectName("S4stopButton")
        self.S4outputLabel = QtWidgets.QLabel(self.previewTab)
        self.S4outputLabel.setGeometry(QtCore.QRect(160, 480, 51, 31))
        self.S4outputLabel.setObjectName("S4outputLabel")
        self.S4outputCombo = QtWidgets.QComboBox(self.previewTab)
        self.S4outputCombo.setGeometry(QtCore.QRect(210, 480, 131, 31))
        self.S4outputCombo.setObjectName("S4outputCombo")
        self.S4outputCombo.addItem("")
        self.S4outputCombo.addItem("")
        self.S4outputCombo.addItem("")
        self.S4outputCombo.addItem("")
        self.S4bgButton = QtWidgets.QPushButton(self.previewTab)
        self.S4bgButton.setGeometry(QtCore.QRect(160, 442, 181, 31))
        self.S4bgButton.setObjectName("S4bgButton")
        #self.S4restartButton = QtWidgets.QPushButton(self.previewTab)
        #self.S4restartButton.setGeometry(QtCore.QRect(450, 440, 181, 31))
        #self.S4restartButton.setObjectName("S4restartButton")
        self.S4backButton = QtWidgets.QPushButton(self.previewTab)
        self.S4backButton.setGeometry(QtCore.QRect(450, 440, 181, 71))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(18)
        self.S4backButton.setFont(font)
        self.S4backButton.setObjectName("S4backButton")
        self.tabWidget.addTab(self.previewTab, "")
        self.detailedTab = QtWidgets.QWidget()
        self.detailedTab.setObjectName("detailedTab")
        self.S5display = QtWidgets.QLabel(self.detailedTab)
        self.S5display.setGeometry(QtCore.QRect(250, 10, 541, 411))
        self.S5display.setAlignment(QtCore.Qt.AlignCenter)
        self.S5display.setObjectName("S5display")
        self.S5display.setScaledContents(True)
        self.S5frame = QtWidgets.QLCDNumber(self.detailedTab)
        self.S5frame.setGeometry(QtCore.QRect(100, 130, 121, 23))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.ToolTipText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.ToolTipText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Button, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Light, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Midlight, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Dark, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Mid, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.BrightText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Window, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Shadow, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.AlternateBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 220))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ToolTipBase, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.ToolTipText, brush)
        brush = QtGui.QBrush(QtGui.QColor(255, 255, 255, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.PlaceholderText, brush)
        self.S5frame.setPalette(palette)
        self.S5frame.setProperty("value", 1.0)
        self.S5frame.setObjectName("S5frame")
        self.S5frameLabel = QtWidgets.QLabel(self.detailedTab)
        self.S5frameLabel.setGeometry(QtCore.QRect(26, 130, 61, 21))
        self.S5frameLabel.setObjectName("S5frameLabel")
        self.S5outputLabel = QtWidgets.QLabel(self.detailedTab)
        self.S5outputLabel.setGeometry(QtCore.QRect(30, 100, 51, 21))
        self.S5outputLabel.setObjectName("S5outputLabel")
        self.S5outputCombo = QtWidgets.QComboBox(self.detailedTab)
        self.S5outputCombo.setGeometry(QtCore.QRect(100, 100, 121, 22))
        self.S5outputCombo.setObjectName("S5outputCombo")
        self.S5outputCombo.addItem("")
        self.S5outputCombo.addItem("")
        self.S5outputCombo.addItem("")
        self.S5outputCombo.addItem("")
        self.S5resultGroup = QtWidgets.QGroupBox(self.detailedTab)
        self.S5resultGroup.setGeometry(QtCore.QRect(10, 420, 311, 91))
        self.S5resultGroup.setObjectName("S5resultGroup")
        self.S5result = QtWidgets.QLabel(self.S5resultGroup)
        self.S5result.setGeometry(QtCore.QRect(30, 30, 251, 51))
        self.S5result.setText("")
        self.S5result.setPixmap(QtGui.QPixmap("results/Nothing.png"))
        self.S5result.setScaledContents(True)
        self.S5result.setAlignment(QtCore.Qt.AlignCenter)
        self.S5result.setObjectName("S5result")
        self.S5detailsGroup = QtWidgets.QGroupBox(self.detailedTab)
        self.S5detailsGroup.setGeometry(QtCore.QRect(20, 160, 211, 151))
        self.S5detailsGroup.setObjectName("S5detailsGroup")
        self.S5legsLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5legsLabel.setGeometry(QtCore.QRect(10, 60, 191, 21))
        self.S5legsLabel.setObjectName("S5legsLabel")
        self.S5cyLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5cyLabel.setGeometry(QtCore.QRect(10, 40, 191, 21))
        self.S5cyLabel.setObjectName("S5cyLabel")
        self.S5cxLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5cxLabel.setGeometry(QtCore.QRect(10, 20, 191, 21))
        self.S5cxLabel.setObjectName("S5cxLabel")
        self.S5wLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5wLabel.setGeometry(QtCore.QRect(10, 100, 191, 21))
        self.S5wLabel.setObjectName("S5wLabel")
        self.S5ellipseLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5ellipseLabel.setGeometry(QtCore.QRect(10, 80, 191, 21))
        self.S5ellipseLabel.setObjectName("S5ellipseLabel")
        self.S5hLabel = QtWidgets.QLabel(self.S5detailsGroup)
        self.S5hLabel.setGeometry(QtCore.QRect(10, 120, 191, 21))
        self.S5hLabel.setObjectName("S5hLabel")
        self.S5previousButton = QtWidgets.QPushButton(self.detailedTab)
        self.S5previousButton.setGeometry(QtCore.QRect(30, 330, 191, 23))
        self.S5previousButton.setObjectName("S5previousButton")
        self.S5nextButton = QtWidgets.QPushButton(self.detailedTab)
        self.S5nextButton.setGeometry(QtCore.QRect(30, 360, 191, 23))
        self.S5nextButton.setObjectName("S5nextButton")
        self.S5firstButton = QtWidgets.QPushButton(self.detailedTab)
        self.S5firstButton.setGeometry(QtCore.QRect(30, 390, 191, 23))
        self.S5firstButton.setObjectName("S5firstButton")
        self.S5logo = QtWidgets.QLabel(self.detailedTab)
        self.S5logo.setGeometry(QtCore.QRect(20, 20, 211, 71))
        self.S5logo.setText("")
        self.S5logo.setPixmap(QtGui.QPixmap("icons/step-4.png"))
        self.S5logo.setScaledContents(True)
        self.S5logo.setAlignment(QtCore.Qt.AlignCenter)
        self.S5logo.setObjectName("S5logo")
        self.S5backButton = QtWidgets.QPushButton(self.detailedTab)
        self.S5backButton.setGeometry(QtCore.QRect(390, 450, 151, 61))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        font.setBold(False)
        font.setWeight(50)
        self.S5backButton.setFont(font)
        self.S5backButton.setObjectName("S5backButton")
        self.S5proceedButton = QtWidgets.QPushButton(self.detailedTab)
        self.S5proceedButton.setGeometry(QtCore.QRect(560, 450, 151, 61))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        self.S5proceedButton.setFont(font)
        self.S5proceedButton.setObjectName("S5proceedButton")
        self.tabWidget.addTab(self.detailedTab, "")
        self.finalTab = QtWidgets.QWidget()
        self.finalTab.setObjectName("finalTab")
        self.S6divider = QtWidgets.QFrame(self.finalTab)
        self.S6divider.setGeometry(QtCore.QRect(390, 30, 20, 371))
        self.S6divider.setFrameShape(QtWidgets.QFrame.VLine)
        self.S6divider.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.S6divider.setObjectName("S6divider")
        self.S6result = QtWidgets.QLabel(self.finalTab)
        self.S6result.setGeometry(QtCore.QRect(20, 60, 351, 71))
        self.S6result.setText("")
        self.S6result.setPixmap(QtGui.QPixmap("results/Nothing.png"))
        self.S6result.setScaledContents(True)
        self.S6result.setAlignment(QtCore.Qt.AlignCenter)
        self.S6result.setObjectName("S6result")
        self.S6precent = QtWidgets.QLabel(self.finalTab)
        self.S6precent.setGeometry(QtCore.QRect(450, 48, 191, 101))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(120, 120, 120))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        self.S6precent.setPalette(palette)
        font = QtGui.QFont()
        font.setFamily("ChunkFive Ex")
        font.setPointSize(72)
        self.S6precent.setFont(font)
        self.S6precent.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.S6precent.setObjectName("S6precent")
        self.S6precentLabel = QtWidgets.QLabel(self.finalTab)
        self.S6precentLabel.setGeometry(QtCore.QRect(650, 70, 81, 61))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(120, 120, 120))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        self.S6precentLabel.setPalette(palette)
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(24)
        self.S6precentLabel.setFont(font)
        self.S6precentLabel.setAlignment(QtCore.Qt.AlignBottom|QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft)
        self.S6precentLabel.setObjectName("S6precentLabel")
        self.S6accuracyLabel = QtWidgets.QLabel(self.finalTab)
        self.S6accuracyLabel.setGeometry(QtCore.QRect(490, 30, 91, 31))
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(12)
        self.S6accuracyLabel.setFont(font)
        self.S6accuracyLabel.setObjectName("S6accuracyLabel")
        self.S6correctGroup = QtWidgets.QGroupBox(self.finalTab)
        self.S6correctGroup.setGeometry(QtCore.QRect(30, 160, 331, 71))
        self.S6correctGroup.setObjectName("S6correctGroup")
        self.S6yesButton = QtWidgets.QPushButton(self.S6correctGroup)
        self.S6yesButton.setGeometry(QtCore.QRect(20, 30, 131, 31))
        self.S6yesButton.setObjectName("S6yesButton")
        self.S6noButton = QtWidgets.QPushButton(self.S6correctGroup)
        self.S6noButton.setGeometry(QtCore.QRect(180, 30, 131, 31))
        self.S6noButton.setObjectName("S6noButton")
        self.S6resultLabel = QtWidgets.QLabel(self.finalTab)
        self.S6resultLabel.setGeometry(QtCore.QRect(30, 30, 171, 31))
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(12)
        self.S6resultLabel.setFont(font)
        self.S6resultLabel.setObjectName("S6resultLabel")
        self.S6correctLabel = QtWidgets.QLabel(self.finalTab)
        self.S6correctLabel.setGeometry(QtCore.QRect(30, 120, 331, 41))
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(12)
        self.S6correctLabel.setFont(font)
        self.S6correctLabel.setAlignment(QtCore.Qt.AlignCenter)
        self.S6correctLabel.setObjectName("S6correctLabel")
        self.S6leftRTF = QtWidgets.QTextBrowser(self.finalTab)
        self.S6leftRTF.setGeometry(QtCore.QRect(30, 240, 341, 161))
        self.S6leftRTF.setObjectName("S6leftRTF")
        self.S6rightRTF = QtWidgets.QTextBrowser(self.finalTab)
        self.S6rightRTF.setGeometry(QtCore.QRect(430, 240, 331, 161))
        self.S6rightRTF.setObjectName("S6rightRTF")
        self.S6tests = QtWidgets.QLabel(self.finalTab)
        self.S6tests.setGeometry(QtCore.QRect(410, 170, 181, 41))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 213, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.WindowText, brush)
        brush = QtGui.QBrush(QtGui.QColor(120, 120, 120))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.WindowText, brush)
        self.S6tests.setPalette(palette)
        font = QtGui.QFont()
        font.setFamily("ChunkFive Ex")
        font.setPointSize(40)
        self.S6tests.setFont(font)
        self.S6tests.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.S6tests.setObjectName("S6tests")
        self.S6testsLabel1 = QtWidgets.QLabel(self.finalTab)
        self.S6testsLabel1.setGeometry(QtCore.QRect(610, 170, 91, 21))
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(12)
        self.S6testsLabel1.setFont(font)
        self.S6testsLabel1.setObjectName("S6testsLabel1")
        self.S6testsLabel2 = QtWidgets.QLabel(self.finalTab)
        self.S6testsLabel2.setGeometry(QtCore.QRect(610, 190, 91, 21))
        font = QtGui.QFont()
        font.setFamily("PRIMETIME")
        font.setPointSize(12)
        self.S6testsLabel2.setFont(font)
        self.S6testsLabel2.setObjectName("S6testsLabel2")
        self.S6proceedButton = QtWidgets.QPushButton(self.finalTab)
        self.S6proceedButton.setGeometry(QtCore.QRect(410, 430, 241, 71))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(14)
        self.S6proceedButton.setFont(font)
        self.S6proceedButton.setObjectName("S6proceedButton")
        self.S6exitButton = QtWidgets.QPushButton(self.finalTab)
        self.S6exitButton.setGeometry(QtCore.QRect(240, 430, 151, 71))
        font = QtGui.QFont()
        font.setFamily("Keep Calm")
        font.setPointSize(22)
        font.setBold(False)
        font.setWeight(50)
        self.S6exitButton.setFont(font)
        self.S6exitButton.setObjectName("S6exitButton")
        self.S6leftGroup = QtWidgets.QGroupBox(self.finalTab)
        self.S6leftGroup.setEnabled(True)
        self.S6leftGroup.setGeometry(QtCore.QRect(10, 10, 381, 411))
        self.S6leftGroup.setObjectName("S6leftGroup")
        self.S6rightGroup = QtWidgets.QGroupBox(self.finalTab)
        self.S6rightGroup.setEnabled(True)
        self.S6rightGroup.setGeometry(QtCore.QRect(409, 10, 381, 411))
        self.S6rightGroup.setObjectName("S6rightGroup")
        self.S6divider.raise_()
        self.S6proceedButton.raise_()
        self.S6exitButton.raise_()
        self.S6leftGroup.raise_()
        self.S6rightGroup.raise_()
        self.S6leftRTF.raise_()
        self.S6correctGroup.raise_()
        self.S6correctLabel.raise_()
        self.S6resultLabel.raise_()
        self.S6result.raise_()
        self.S6rightRTF.raise_()
        self.S6testsLabel2.raise_()
        self.S6precent.raise_()
        self.S6tests.raise_()
        self.S6accuracyLabel.raise_()
        self.S6testsLabel1.raise_()
        self.S6precentLabel.raise_()
        self.tabWidget.addTab(self.finalTab, "")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        _translate = QtCore.QCoreApplication.translate

        self.settingsTab.setEnabled(False)
        self.previewTab.setEnabled(False)
        self.detailedTab.setEnabled(False)
        self.finalTab.setEnabled(False)

        self.S1proceedButton.clicked.connect(self.S1proceed)
        self.S6proceedButton.clicked.connect(self.S1proceed)
        self.S1exitButton.clicked.connect(QtWidgets.qApp.quit)
        self.S6exitButton.clicked.connect(QtWidgets.qApp.quit)
        self.S2openButton.clicked.connect(self.S2open)
        self.S2proceedButton.clicked.connect(self.S2proceed)
        self.S2backButton.clicked.connect(self.S2back)
        self.S2testCombo.currentIndexChanged.connect(self.S2test)
        self.S3backButton.clicked.connect(self.S3back)
        self.S3proceedButton.clicked.connect(self.S3proceed)
        self.S3defaultButton.clicked.connect(self.S3default)
        self.S4backButton.clicked.connect(self.S4back)
        self.S4stopButton.clicked.connect(self.S4stop)
        #self.S4restartButton.clicked.connect(self.S4restart)
        self.S4bgButton.clicked.connect(self.S4bg)
        self.S5backButton.clicked.connect(self.S5back)
        self.S5nextButton.clicked.connect(self.S5next)
        self.S5previousButton.clicked.connect(self.S5previous)
        self.S5firstButton.clicked.connect(self.S5first)
        self.S5outputCombo.currentIndexChanged.connect(self.S5switch)
        self.S5proceedButton.clicked.connect(self.S5proceed)
        self.S6yesButton.clicked.connect(self.S6yes)
        self.S6noButton.clicked.connect(self.S6no)
        self.S6proceedButton.clicked.connect(self.S1proceed)
        self.S2webcamRadio.toggled.connect(lambda: self.S2status.setText(_translate("MainWindow", "Chosen method: Webcam")))
        self.S2videoRadio.toggled.connect(lambda: self.S2status.setText(_translate("MainWindow", "Chosen method: Video")))
        self.S2testRadio.toggled.connect(lambda: self.S2status.setText(_translate("MainWindow", "Chosen Test: "+self.S2testCombo.currentText())))

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Reconnaisseur de Mouvement"))
        self.S1exitButton.setText(_translate("MainWindow", "Exit"))
        self.S1proceedButton.setText(_translate("MainWindow", "Proceed"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.mainTab), _translate("MainWindow", "Main"))
        self.S2webcamRadio.setText(_translate("MainWindow", "Webcam"))
        self.S2videoRadio.setText(_translate("MainWindow", "Open a video file"))
        self.S2testRadio.setText(_translate("MainWindow", "Ready datasets"))
        self.S2testCombo.setItemText(0, _translate("MainWindow", "Test 1"))
        self.S2testCombo.setItemText(1, _translate("MainWindow", "Test 2"))
        self.S2testCombo.setItemText(2, _translate("MainWindow", "Test 3"))
        self.S2testCombo.setItemText(3, _translate("MainWindow", "Test 4"))
        self.S2testCombo.setItemText(4, _translate("MainWindow", "Test 5"))
        self.S2openButton.setText(_translate("MainWindow", "Open"))
        self.S2status.setText(_translate("MainWindow", "Chosen method: Webcam"))
        self.S2backButton.setText(_translate("MainWindow", "Back"))
        self.S2proceedButton.setText(_translate("MainWindow", "Proceed"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.inputTab), _translate("MainWindow", "Input"))
        self.S3groupBox.setTitle(_translate("MainWindow", "Frame processing"))
        self.S3gbLabel.setText(_translate("MainWindow", "Gaussian Blur Intensity"))
        self.S3biLabel.setText(_translate("MainWindow", "Binarizing Threshold"))
        self.S3dilateLabel.setText(_translate("MainWindow", "Dilation Intensity"))
        self.S3outputCombo.setItemText(0, _translate("MainWindow", "RGB Image"))
        self.S3outputCombo.setItemText(1, _translate("MainWindow", "Grayscale+Gaussian Blur"))
        self.S3outputCombo.setItemText(2, _translate("MainWindow", "Background Subtraction"))
        self.S3outputCombo.setItemText(3, _translate("MainWindow", "Silhouette"))
        self.S3outputLabel.setText(_translate("MainWindow", "Output:"))
        self.S3groupBox_2.setTitle(_translate("MainWindow", "Silhouette analysis"))
        self.S3legsLabel.setText(_translate("MainWindow", "Legs Angle Threshold"))
        self.S3walkLabel.setText(_translate("MainWindow", "Centroid Walking Threshold"))
        self.S3runLabel.setText(_translate("MainWindow", "Centroid Running Threshold"))
        self.S3ellipseLabel.setText(_translate("MainWindow", "Ellipse Angle Threshold"))
        self.S3defaultButton.setText(_translate("MainWindow", "Default Values"))
        self.S3backButton.setText(_translate("MainWindow", "Back"))
        self.S3proceedButton.setText(_translate("MainWindow", "Proceed"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.settingsTab), _translate("MainWindow", "Settings"))
        self.S4display.setText(_translate("MainWindow", "No input"))
        self.S4stopButton.setText(_translate("MainWindow", "Stop"))
        self.S4outputLabel.setText(_translate("MainWindow", "Output:"))
        self.S4outputCombo.setItemText(0, _translate("MainWindow", "RGB Image"))
        self.S4outputCombo.setItemText(1, _translate("MainWindow", "Grayscale+Gaussian Blur"))
        self.S4outputCombo.setItemText(2, _translate("MainWindow", "Background Subtraction"))
        self.S4outputCombo.setItemText(3, _translate("MainWindow", "Silhouette"))
        self.S4bgButton.setText(_translate("MainWindow", "Save Background Frame"))
        #self.S4restartButton.setText(_translate("MainWindow", "Restart"))
        self.S4backButton.setText(_translate("MainWindow", "Back"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.previewTab), _translate("MainWindow", "Preview"))
        self.S5display.setText(_translate("MainWindow", "No input"))
        self.S5frameLabel.setText(_translate("MainWindow", "Frame N°:"))
        self.S5outputLabel.setText(_translate("MainWindow", "Output:"))
        self.S5outputCombo.setItemText(0, _translate("MainWindow", "RGB Image"))
        self.S5outputCombo.setItemText(1, _translate("MainWindow", "Grayscale+Gaussian Blur"))
        self.S5outputCombo.setItemText(2, _translate("MainWindow", "Background Subtraction"))
        self.S5outputCombo.setItemText(3, _translate("MainWindow", "Silhouette"))
        self.S5resultGroup.setTitle(_translate("MainWindow", "Recognized Move"))
        self.S5detailsGroup.setTitle(_translate("MainWindow", "Silhouette Analysis Results"))
        self.S5legsLabel.setText(_translate("MainWindow", "Angle Between Legs = 0"))
        self.S5cyLabel.setText(_translate("MainWindow", "Centroid\'s Y Axis Difference = 0"))
        self.S5cxLabel.setText(_translate("MainWindow", "Centroid\'s X Axis Difference = 0"))
        self.S5wLabel.setText(_translate("MainWindow", "Bounding Rectangle\'s Width = 0"))
        self.S5ellipseLabel.setText(_translate("MainWindow", "Bounding Ellipse\'s Angle = 0"))
        self.S5hLabel.setText(_translate("MainWindow", "Bounding Rectangle\'s Height = 0"))
        self.S5previousButton.setText(_translate("MainWindow", "Previous Frame"))
        self.S5nextButton.setText(_translate("MainWindow", "Next Frame"))
        self.S5firstButton.setText(_translate("MainWindow", "First Frame"))
        self.S5backButton.setText(_translate("MainWindow", "Back"))
        self.S5proceedButton.setText(_translate("MainWindow", "Proceed"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.detailedTab), _translate("MainWindow", "Detailed Results"))
        self.S6precent.setText(_translate("MainWindow", "100"))
        self.S6precentLabel.setText(_translate("MainWindow", "%"))
        self.S6accuracyLabel.setText(_translate("MainWindow", "Accuracy:"))
        self.S6correctGroup.setTitle(_translate("MainWindow", "Was the recognition correct?"))
        self.S6yesButton.setText(_translate("MainWindow", "Yes"))
        self.S6noButton.setText(_translate("MainWindow", "No"))
        self.S6resultLabel.setText(_translate("MainWindow", "DOMINANT MOVE:"))
        self.S6correctLabel.setText(_translate("MainWindow", "CORRECT RECOGNITION"))
        self.S6leftRTF.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\">All Recognized moves:</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:14pt; font-weight:600;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Standing (0% | 0 out of 0 frames)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Walking (0% | 0 out of 0 frames)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Running (0% | 0 out of 0 frames)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Sitting (0% | 0 out of 0 frames)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Falling (0% | 0 out of 0 frames)</span></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:12pt;\">- Laying (0% | 0 out of 0 frames)</span></p></body></html>"))
        self.S6rightRTF.setHtml(_translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'MS Shell Dlg 2\'; font-size:10pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:14pt; font-weight:600;\">All tests:</span></p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-size:14pt; font-weight:600;\"><br /></p>\n</body></html>"))
        self.S6tests.setText(_translate("MainWindow", "0/0"))
        self.S6testsLabel1.setText(_translate("MainWindow", "CORRECT"))
        self.S6testsLabel2.setText(_translate("MainWindow", "RESULTS"))
        self.S6proceedButton.setText(_translate("MainWindow", "Launch Another Test"))
        self.S6exitButton.setText(_translate("MainWindow", "Exit"))
        self.S6leftGroup.setTitle(_translate("MainWindow", "Current Video"))
        self.S6rightGroup.setTitle(_translate("MainWindow", "All Tests"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.finalTab), _translate("MainWindow", "Final Results"))

if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())

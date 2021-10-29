from rplidar import RPLidar
import rplidar
import matplotlib.pyplot as plt
import math
import time
import threading
import numpy as np
from openal.audio import SoundSink, SoundSource, SoundData
from openal.loaders import load_wav_file, load_file
from direct.showbase import Audio3DManager
import heapq

lidar = RPLidar('/dev/ttyUSB0', timeout=10)

#info = lidar.get_info()
#print(info)
sink = SoundSink()
sink.activate()
primeSource = SoundSource(position=[0, 0, 0])
primeSource.looping = False
primeData = load_file("./beep.wav")
primeSource.queue(primeData)
sink.play(primeSource)

sourceList = []
for iy in range(10):
    sources = SoundSource(position=[0, 0, 0])
    sources.looping = False
    datas = load_file("./waveFiles/wave0.wav")
    sources.queue(datas)
    sink.play(sources)
    sourceList.append(sources)
def startSource(fre, x, y, sour):
    data = load_file("./waveFiles/wave" + str(fre) + ".wav")
    sour.position = [x, y, 0]
    sour.queue(data)


#health = lidar.get_health()
#print(health)
print("Speed set")
iterator = lidar.iter_scans(max_buf_meas=False)
lidar.motor_speed = rplidar.MAX_MOTOR_PWM
#iterator = lidar.iter_scans(max_buf_meas=False)

#Use dot product to find angle between points
def runScan():
    try:
        lidar.clean_input()
        scan = next(iterator)
        angList = []
        disList = []
        xList = []
        yList = []
        for t in scan:
            angList.append(t[1])
            disList.append(t[2])
            x_first = math.sin(math.radians(t[1]))
            y_first = math.cos(math.radians(t[1]))
            magnitude = t[2] / 100
            x_real = x_first * magnitude
            y_real = y_first * magnitude
            xList.append(x_real)
            yList.append(y_real)
        return angList, disList, xList, yList
    except Exception as e:
        print(e)

plt.ion()
figure, ax = plt.subplots(figsize=(8,6))
rm = ax.scatter([1, 1], [1, 1])
rm2 = ax.scatter([1, 1], [1, 1])
rm3 = ax.scatter([0], [0], s=100, color="green")
rm4 = ax.scatter([0], [0], s=100, color="black")
point1 = [10, 1]
point2 = [-1, -10]
figure.canvas.draw()


def absDist(vec1, vec2):
    distSum = 0
    for hu in zip(vec1, vec2):
        dif = (hu[1] - hu[0]) ** 2
        distSum += dif
    finalAns = math.sqrt(distSum)
    return abs(finalAns)

lidar.get_info()

while True:
    rm.remove()
    rm2.remove()
    rm3.remove()
    b = runScan()
    angList = b[0]
    xList = b[2]
    yList = b[3]
    wallXList = []
    wallYList = []

    #Find line between two points, see how many dots intersect
    distanceList = []
    dotsDot = []
    cosList = []
    totalXs = []
    totalYs = []
    l = 0
    for l, mV in enumerate(zip(xList, yList)):
        if l > 10:
            #firstSlope = (yList[l] - yList[l - 1]) / (xList[l] - xList[l - 1])
            als = (yList[l] - yList[l - 10])
            bs = (-1) * (xList[l] - xList[l - 10])
            sl = 0 - ((als * xList[l]) + (bs * yList[l]))
            jAv = 0
            closeNums = 0
            xLists = []
            yLists = []
            dots = []
            for mt in zip(xList, yList):
                numr = abs((als * mt[0]) + (bs * mt[1]) + sl)
                denR = math.sqrt((als**2) + (bs**2))
                distance = numr / denR
                if distance < 0.075:
                    xLists.append(mt[0])
                    yLists.append(mt[1])
                    jAv += 1
            totalXs.append(xLists)
            totalYs.append(yLists)
            distanceList.append(jAv)
    #closeIndL = list(map(min, zip(distanceList, cosList)))
    if len(distanceList) > 50:
        #print(distanceList)
        maxX = distanceList.index(max(distanceList))
        xPss = totalXs[maxX]
        yPss = totalYs[maxX]
        xValueMain = xList[maxX]
        yValueMain = yList[maxX]
        nLarg = heapq.nlargest(1, distanceList)
        for nLs in nLarg:
            closeInd = distanceList.index(nLs)
            distanceList.remove(distanceList[closeInd])
            #print(distanceList)
            xP = totalXs[closeInd]
            yP = totalYs[closeInd]
            for xy in zip(xP, yP):
                if xy[0] in xList:
                    remInd = xList.index(xy[0])
                    angList.remove(angList[remInd])
                    xList.remove(xy[0])
                if xy[1] in yList:
                    yList.remove(xy[1])
        """print(xList)
        print("Y")
        print(yList)
        print("A")
        print(angList)
        print(" ")"""
        distanceFrom = []
        for j in zip(xList, yList):
            rooted = math.sqrt((j[0] ** 2) + (j[1] ** 2))
            distanceFrom.append(rooted)
        #mp = mostPoints(xList, yList, angList)
        smallest_nums = heapq.nsmallest(10, distanceFrom)
        centreXVals = []
        centreYVals = []
        centreAngles = []
        for p in smallest_nums:
            smallInd = distanceFrom.index(p)
            centreXVals.append(xList[smallInd])
            centreYVals.append(yList[smallInd])
            centreAngles.append(angList[smallInd])

        disRange = 4
        allRange = 35
        lenList = []
        for tu in zip(centreXVals, centreYVals, centreAngles):
            firstAng = tu[2]
            d1 = math.sqrt((tu[0] ** 2) + (tu[1] ** 2))
            newXs = []
            newYs = []
            numPoints = 0
            for ut in zip(centreXVals, centreYVals, centreAngles):
                d2 = math.sqrt((ut[0] ** 2) + (ut[1] ** 2))
                secondAng = ut[2]
                totDist = abs(secondAng - firstAng)
                absolute = absDist((tu[0], tu[1]), (ut[0], ut[1]))
                if absolute < disRange and totDist < allRange and tu[0] != ut[0] and tu[1] != ut[1]:
                    centreXVals.remove(ut[0])
                    centreYVals.remove(ut[1])
                    centreAngles.remove(ut[2])
        finalX = []
        finalY = []
        #print(len(centreXVals), len(centreYVals))
        #print(centreYVals)
        centFinalsX = []
        centFinalsY = []
        lenLists = []
        centreLists = []
        for iuo in zip(centreXVals, centreYVals, centreAngles):
            firstAng = iuo[2]
            d1 = math.sqrt((iuo[0] ** 2) + (iuo[1] ** 2))
            curFinX = []
            curFinY = []
            for kio in zip(xList, yList, angList):
                d2 = math.sqrt((kio[0] ** 2) + (kio[1] ** 2))
                secondAng = kio[2]
                totDist = abs(secondAng - firstAng)
                absolute = absDist((iuo[0], iuo[1]), (kio[0], kio[1]))
                if absolute < disRange and iuo[0] != kio[0] and iuo[1] != kio[1]:
                    curFinX.append(kio[0])
                    curFinY.append(kio[1])
            #print(curFinX)
            #print(curFinY)
            if len(curFinX) > 0:
                #mDis = math.sqrt(((curFinX[round(len(curFinX) / 2)]) ** 2) + ((curFinY[round(len(curFinY) / 2)]) ** 2))
                centFinalsX.append(curFinX[round(len(curFinX) / 2)])
                centFinalsY.append(curFinY[round(len(curFinY) / 2)])
                finalX.append(curFinX)
                finalY.append(curFinY)
                lenLists.append(len(curFinX))
        print(lenLists)
        """smallest_final = heapq.nsmallest(2, centreLists)
        bigX = []
        bigY = []
        bigL = []
        for uyt in zip(centreLists, centFinalsX, centFinalsY, lenLists):
            if uyt[0] in smallest_final:
                bigX.append(uyt[1])
                bigY.append(uyt[2])
                bigL.append(uyt[3])"""
        #print(len(lenLists), len(centFinalsX), len(centFinalsY))
        wallX = round(xValueMain, 2)
        wallY = round(yValueMain, 2)
        primeSource.position = [wallX, wallY, 0]
        for lna, ioSo in enumerate(sourceList):
            if lna <= (len(lenLists) - 1):
                startSource(lenLists[lna], centFinalsX[lna], centFinalsY[lna], ioSo)
            else:
                startSource(0, 0, 0, ioSo)
        sink.play(primeSource)
        firsT = time.time()
        currT = 0
        while (currT - firsT) < 0.05:
            sink.update()
            currT = time.time()
        rm = ax.scatter(xList, yList, s=35, color=(0, 0, 0.99))
        rm3 = ax.scatter(centFinalsX, centFinalsY, s=lenLists, color=(0, 0.99, 0.99))
        rm2 = ax.scatter([wallX], [wallY], s=50, color=(0.99, 0, 0))
    else:
        rm = ax.scatter([1], [1], s=30, color=(0, 0, 0.99))
        rm3 = ax.scatter([1], [1], s=40, color=(0, 0.99, 0.99))
        rm2 = ax.scatter([1], [1], s=50, color=(0.99, 0, 0))
    figure.canvas.draw()
    figure.canvas.flush_events()

print("   ")
print("Closing")
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
print("Closed")
print("   ")

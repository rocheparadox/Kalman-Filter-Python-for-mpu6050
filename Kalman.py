class Kalman:
    def __init__():
        this.QAngle = 0.001
        this.QBias = 0.003
        this.RMeasure = 0.03
        this.angle = 0.0
        this.bias = 0.0
        this.rate = 0.0
        this.p=[[0.0,0.0],[0.0,0.0]]

    '''def kalman():
        QAngle = 0.001
        QBias = 0.003
        RMeasure = 0.03

        angle = 0.0
        bias = 0.0

        p[0][0] = 0.0
        p[0][1] = 0.0
        p[1][0] = 0.0
        p[1][1] = 0.0'''

    def getAngle(newAngle, newRate,dt):
        #step 1:
        this.rate = newRate - this.bias;    #new_rate is the latest Gyro measurement
        this.angle += dt * this.rate;

        #Step 2:
        this.p[0][0] += dt * (dt*this.p[1][1] -this.p[0][1] - this.p[1][0] + this.QAngle)
        this.p[0][1] -= dt * this.p[1][1]
        this.p[1][0] -= dt * this.p[1][1]
        this.p[1][1] += this.QBias * dt

        #Step 3: Innovation
        y = newAngle - this.angle

        #Step 4: Innovation covariance
        s = this.p[0][0] + this.RMeasure

        #Step 5:    Kalman Gain
        k[0] = this.p[0][0]/s
        k[1] = this.p[1][0]/s

        #Step 6: Update the Angle
        angle += k[0] * y
        this.bias  += k[1] * y

        #Step 7: Calculate estimation error covariance - Update the error covariance
        p00Temp = this.p[0][0]
        p01Temp = this.p[0][1]

        this.P[0][0] -= K[0] * P00Temp;
        this.P[0][1] -= K[0] * P01Temp;
        this.P[1][0] -= K[1] * P00Temp;
        this.P[1][1] -= K[1] * P01Temp;

        return Angle

    def setAngle(angle):
        this.angle = angle

    def setQAngle(QAngle):
        this.QAngle = QAngle

    def setQBias(QBias):
        this.QBias = QBias

    def setRMeasure(RMeasure):
        this.RMeasure = RMeasure

    def getRate():
        return this.rate

    def getQAngle():
        return this.QAngle

    def getQBias():
        return this.QBias

    def  getRMeasure():
        return this.RMeasure

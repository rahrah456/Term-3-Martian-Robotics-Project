// ============================================================
//   WALL FOLLOWING  —  cascaded PID
//   Sensor mounted at 58° to wall (right side assumed)
//   d_perp = d_measured * cos(58°) = d_measured * 0.5299
// ============================================================

// Outer loop: wall distance PID
// Tune these independently from PIDSpeed gains
float wKp = 1.5f, wKi = 0.0f, wKd = 0.0f;
float wIntegral   = 0.0f;
float wPrevError  = 0.0f;
unsigned long wPrevTime = 0;

const float WALL_SETPOINT_CM = 20.0f;   // desired perp distance
const float COS58 = 0.5299f;
const float MAX_CORRECTION = 300.0f;    // clamp — tune this
const float WINDUP_LIMIT   = 150.0f;

float wallPID(float dMeasured) {
    float dPerp = dMeasured * COS58;
    float error = dPerp - WALL_SETPOINT_CM;

    unsigned long now = millis();
    float dt = (now - wPrevTime) / 1000.0f;
    wPrevTime = now;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.02f;  // guard bad dt on first call

    // Integral with anti-windup
    if (abs(error) < 10.0f)
        wIntegral += error * dt;
    wIntegral = constrain(wIntegral, -WINDUP_LIMIT, WINDUP_LIMIT);

    float derivative = (error - wPrevError) / dt;
    wPrevError = error;

    float correction = wKp*error + wKi*wIntegral + wKd*derivative;
    return constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);
}

// Median filter — rejects UDS spikes from oblique angle
float medianOf5(float a,float b,float c,float d,float e){
    float v[5]={a,b,c,d,e};
    // simple insertion sort on 5 elements
    for(int i=1;i<5;i++){
        float key=v[i]; int j=i-1;
        while(j>=0 && v[j]>key){ v[j+1]=v[j]; j--; }
        v[j+1]=key;
    }
    return v[2];
}

void wallFollow(int baseSpeed) {
    pidL.reset(encL);
    pidR.reset(encR);
    wIntegral = 0; wPrevError = 0;
    wPrevTime = millis();

    float buf[5] = {0}; int bufIdx = 0; bool bufFull = false;

    Serial.println("Wall following — send any char to stop");

    while (true) {
        if (handleEStop()) { setMotors(0, 0); waitForUnkill(); break; }
        if (Serial.available())  { setMotors(0, 0); break; }

        // --- Sensor read with median filter ---
        buf[bufIdx] = readUDS();            // your existing UDS read
        bufIdx = (bufIdx + 1) % 5;
        if (bufIdx == 0) bufFull = true;

        float dist;
        if (bufFull)
            dist = medianOf5(buf[0],buf[1],buf[2],buf[3],buf[4]);
        else
            dist = buf[max(0, bufIdx-1)];   // use latest until buffer fills

        // --- Outer wall PID ---
        float correction = wallPID(dist);

        // correction > 0 → too far from wall → steer right (toward wall)
        // correction < 0 → too close         → steer left  (away from wall)
        // Assumes sensor is on the RIGHT side; swap signs if on left
        int speedL = baseSpeed + (int)correction;
        int speedR = baseSpeed - (int)correction;

        // Prevent either tread reversing unintentionally
        speedL = constrain(speedL, 0, 1023);
        speedR = constrain(speedR, 0, 1023);

        // --- Inner velocity PIDs (unchanged) ---
        int cmdL = pidL.update(encL, speedL);
        int cmdR = pidR.update(encR, speedR);
        setMotors(cmdL, cmdR);

        // --- Debug ---
        Serial.print("dRaw:"); Serial.print(dist,1);
        Serial.print(" dPerp:"); Serial.print(dist*COS58,1);
        Serial.print(" err:"); Serial.print(dist*COS58 - WALL_SETPOINT_CM,1);
        Serial.print(" corr:"); Serial.print(correction,1);
        Serial.print(" L:"); Serial.print(speedL);
        Serial.print(" R:"); Serial.println(speedR);

        delay(20);   // 50 Hz — matches your existing loop rate
    }
    setMotors(0, 0);
}
function timeFlag=atTime(timePoints)
    global t dt
    timeFlag=any(abs(t-timePoints)<(dt/2));
end
clear all
close all

s = serial('/dev/cu.usbmodem25909801','BaudRate',115200);
s.Timeout = 5;
fopen(s);

subplot(3,1,1);
pitch = animatedline('Color',[0 0.4470 0.7410]);
pitch_rate = animatedline('Color',[0.8500 0.3250 0.0980]);
roll = animatedline('Color',[0.4940 0.1840 0.5560]);
roll_rate = animatedline('Color',[0.4660 0.6740 0.1880]);
legend('pitch', 'pitchRate', 'roll', 'rollRate');
state_ref = animatedline('LineStyle','--');

subplot(3,1,2);
pitch_motor_vel = animatedline('Color',[0.9290 0.6940 0.1250]);
roll_motor_vel = animatedline('Color',[0.3010 0.7450 0.9330]);
q_pitch = animatedline('Color',[0 0.4470 0.7410]);
q_roll = animatedline('Color',[0.8500 0.3250 0.0980]);
legend('pitchMotorVel', 'rollMotorVel', 'qPitch', 'qRoll');
state_ref2 = animatedline('LineStyle','--');

subplot(3,1,3);
cmd_roll_motor_torque = animatedline('Color',[0 0.4470 0.7410]);
cmd_pitch_motor_torque = animatedline('Color',[0.8500 0.3250 0.0980]);
meas_roll_motor_torque = animatedline('Color',[0.9290 0.6940 0.1250]);
meas_pitch_motor_torque = animatedline('Color',[0.4940 0.1840 0.5560]);
legend('cmdRollTorque', 'cmdPitchTorque', 'measRollTorque', 'measPitchTorque');
torque_ref = animatedline('LineStyle','--');

flushoutput(s);
flushinput(s);
tstart = posixtime(datetime);
 
while true
    t = posixtime(datetime) - tstart;
    serialData = fread(s,12,'float');
    if (length(serialData) == 12)
        display([serialData(1), serialData(5)])
        addpoints(pitch, t, serialData(1));
        addpoints(pitch_rate, t, serialData(2));
        addpoints(pitch_motor_vel, t, serialData(3));
        addpoints(q_pitch, t, serialData(4));
        addpoints(roll, t, serialData(5));
        addpoints(roll_rate, t, serialData(6));
        addpoints(roll_motor_vel, t, serialData(7));
        addpoints(q_roll, t, serialData(8));
        addpoints(cmd_roll_motor_torque, t, serialData(9));
        addpoints(cmd_pitch_motor_torque, t, serialData(10));
        addpoints(meas_roll_motor_torque, t, serialData(11));
        addpoints(meas_pitch_motor_torque, t, serialData(12));
        
        addpoints(state_ref, t, 0);
        addpoints(torque_ref, t, 0);
        addpoints(state_ref2, t, 0);
        drawnow limitrate
    else
        break
    end
end

fclose(s);



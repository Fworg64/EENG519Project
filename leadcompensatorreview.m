[A, B, C, D] = tf2ss([1,4],[1,20]);

left_cmd = .5 - .5*exp(-5*time_to_solve);
right_cmd = .2 - .2*exp(-2*time_to_solve);
c2d(ss(A,B,C,D), .02)


state = 0;
output_record = zeros(1,length(time_to_solve));
for index = 1:length(time_to_solve)
    state = .6703 * state + .01648 * left_cmd(index);
    output_record(1,index) = 5*(-16*state + 1*left_cmd(index));
end

figure();
hold on;
plot(time_to_solve, output_record, time_to_solve, left_cmd);
legend('system output', 'system input');
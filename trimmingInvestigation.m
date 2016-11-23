%EXploring resistor values for trimming a joystick pot
%Jak_o_Shadows
%23-11-2016


close all
clear all
clc

%%

%Number of options to look at for R5 and trimmer pot X
numR5 = 100;
numX = 100;
numY = 20;
%max value/resistor of the two pots
N = 5e5; %ohm
M = 5e5; %ohm
%The standalone resistor
R5 = linspace(1, 10e5, numR5)';
R5 = repmat(R5, 1, numX);

x = linspace(0, 1, numX);
x = repmat(x, numR5, 1);
yList = linspace(0.000, 0.999, numY);

Vsource = 3.3; %V

fname = 'trimInvestigation.gif';

for index=1:length(yList)
    y = yList(index);
    
    
    R1 = x*N;
    R2 = (1-x)*N;
    R3 = y*M;
    R4 = (1-y)*M;

    R1star = 1./(1./R5 + 1./(R2+R3));
    i = Vsource./(R1 + R4 + R1star);

    VBG = i.*R4;
    VAB = i.*(R1 + R1star);

    X = x(1, :);
    Y = R5(:, 1);
    %% Plot it
    surf(X, Y, VBG);
    drawnow
    xlabel('Varying trim pot value x');
    ylabel('R5 Value (ohm)');
    zlabel('Voltage V_{BG}');
    title(['Effect of varying trimmer value for main pot value y = ', num2str(y)]);
    zlim([0, Vsource])
    xlim([0, 1]);
    ylim([0, 10e5]);
    view(45, 30);
    saveas(gcf, ['trim', num2str(index), '.png']);
    %%Save it as a gif
    frame = getframe(1);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(frame.cdata, 256);%im, 256);
    
    if (index==1)
        imwrite(imind, cm, fname, 'gif', 'Loopcount', inf);
    else
        imwrite(imind, cm, fname, 'gif', 'WriteMode', 'append');
    end
    
end









%%%
a = csvread('result.csv',0,0);
SUM0 = a(:,3);
SUM2 = a(:,4);
TH = a(:,5);
diff = a(:,6);
hold on;
set(gcf,'Position',[100 100 1024 768]);
set(gca,'Position',[.15 .15 .80 .80]);
figure_FontSize=8;
plot(SUM0, 'r');
plot(SUM2, 'g');
plot(TH, 'b');
plot(diff, 'c');
legend('SUM0','SUM2', 'TH', 'diff');
figure(1);
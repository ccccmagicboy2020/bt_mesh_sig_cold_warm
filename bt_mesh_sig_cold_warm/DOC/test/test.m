%%%
a = csvread('result.csv',0,0);
SUM0 = a(:,3);
SUM2 = a(:,4);
TH = a(:,5);
diff = a(:,6);
[pks, locs] = findpeaks(SUM2, 'Threshold',1000000);
hold on;
set(gcf,'Position',[100 100 1024 768]);
set(gca,'Position',[.15 .15 .80 .80]);
figure_FontSize=8;
plot(SUM0, 'g');
plot(SUM2, 'r');
plot(TH, 'b');
plot(diff, 'c');
plot(locs, pks, "^r")
legend('SUM0','SUM2', 'TH', 'diff');
text(locs+.02,pks,num2str((1:numel(pks))'))
hold off;
figure(1);
%%%
a = csvread('result.csv',0,0);
SUM0 = a(:,3);
SUM2 = a(:,4);
TH = a(:,5);
diff = a(:,6);
[pks, locs] = findpeaks(diff, 'MinPeakDistance',10, 'minpeakheight', mean(TH));
hold on;
set(gcf,'Position',[100 100 1024 768]);
set(gca,'Position',[.15 .15 .80 .80]);
plot(SUM0, 'g');
plot(SUM2, 'r');
plot(TH, 'b', 'linewidth',1.5);
plot(diff, 'c')
plot(locs, pks, "^r")
legend('SUM0','SUM2', 'TH', 'diff');
text(locs+8,pks,num2str(pks))
hold off;
axis tight;
figure(1);
saveas(gcf, 'test.png');



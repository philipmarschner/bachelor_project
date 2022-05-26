%Parameters
mapPath = "45_deg_kval.png";
dist = 10;
r = 30;
lineWidth = 3;



%Code
I = imread(mapPath);
I = rgb2gray(I);
se = strel('cube',2);
%I = imdilate(I,se);
BW = edge(I,'canny');
[H,theta,rho] = hough(BW);

figure
imshow(imadjust(rescale(H)),[],...
       'XData',theta,...
       'YData',rho,...
       'InitialMagnification','fit');
xlabel('\theta (degrees)')
ylabel('\rho')
title('Hough space')
axis on
axis normal 
hold on
%colormap(gca,hot)
P = houghpeaks(H,30,'threshold',ceil(0.2*max(H(:))));

x = theta(P(:,2)); y = rho(P(:,1));
plot(x,y,'o','color','red','MarkerSize',7,'MarkerFaceColor','red');
legend('Peaks')
figure

subplot(1,2,1)
imshow(BW)
title('Canny edge image')
subplot(1,2,2)
imshow(I)
title('Original map')


clc
clear all
close all

%create a face detector
faceDetector =vision.CascadeObjectDetector;

faceDetector.MergeThreshold=16;

%create an eye detector
eyedetector = vision.CascadeObjectDetector('RightEye','MergeThreshold',8); %create detector for eyepair

%read image                               
%img=imread('visionteam.jpg');
%img=imread('a2.png');

img=imread('222.jpg');
%img=rgb2gray(img);

%Read captions images
right=imread('RIGHT.jpg');
left=imread('LEFT.jpg');
noface=imread('no_face.jpg');
straight=imread('STRAIGHT.jpg');


%display image
figure(1)
imshow(img);
title('Original image')
 
%detect faces 
boundingboxes=step(faceDetector,img);

%Draw boxes around detected faces and display result
img_faces= insertShape(img, 'Rectangle', boundingboxes,'color','r');

%detect faces
figure(2)
imshow(img);
title('Detected faces')
hold on
for i=1:size(boundingboxes,1)
    rectangle('Position',boundingboxes(i,:),'LineWidth',3,...
              'LineStyle','-','EdgeColor','r'); 
end


%count the number of faces detected
Num_faces=length(boundingboxes(:,1));

fprintf('Number of faces detected %d\n',Num_faces)
%%crop the faces from the image
for i=1:Num_faces
  faceImage{i} = imcrop(img,boundingboxes(i,:)); % extract the face from the image
end   

%display the cropped faces
figure(3)
montage(faceImage)
title('Cropped faces')

% Creating bounding box for each of the faces using eyedetector 
count=1;


for i=1:Num_faces
    eyebox = step(eyedetector,faceImage{i});
    
    if isempty(eyebox) || size(eyebox,1)<2
   %if the eye boundary box is empty  or if only one eye is detected  
    
    else
        bboxeye{count}=eyebox;
        facedetectedeye{count}=faceImage{i};
      
        count=count+1;
    end
end    


 %count the number of eye pair detected
 NumeyeDetected=count-1;
 
 fprintf('Number of eye pair detected %d\n',NumeyeDetected)
 
 %crop detected eye from each of the faces 
 count=0;
 for i=1:NumeyeDetected
      
    %crop the eyepair from the face image
    image=  facedetectedeye{i};
    box=bboxeye{i};
    
    %crop the half eyepair from the face image
    eyesImage{count+1} = imcrop(image,box(1,:));   
    %adjust contrast
    eyesImage{count+1} = imadjustn(eyesImage{count+1});         
    
    %crop the half eyepair from the face image
    eyesImage2{count+1} = imcrop(image,box(2,:));    
    %adjust contrast
    eyesImage2{count+1} = imadjustn(eyesImage2{count+1});  

   
    count=count+1;    
  
     
 end


 %display the eye detected
 figure(4)
 montage(eyesImage)
 title('cropped detected eyes')
 
 count=0;

 %loop throught the detected eye pair
for i=1: NumeyeDetected
    
 if isempty(bboxeye{i})
  %if eye boundary box is empty
 else
     
     
  beye=bboxeye{i};
  r = beye(1,4)/4;
  f1 = floor(r-r/3);
  f2 = floor(r+r/2);
  
  %find dark circle from cropped eye using hough transform
  [center, radius, metric] = imfindcircles(eyesImage{count+1}, [f1 f2], 'ObjectPolarity','dark',...
      'Sensitivity', 0.90,'Method','twostage'); 
  
      
   if size(center,1) > 1
     centers{count+1}=mean(center);
     radii{count+1}=mean(radius);
   else
     centers{count+1}=center;
     radii{count+1}=radius; 
   end   
     
      r =  beye(2,4)/4;
      f1 = floor(r-r/2);
      f2 = floor(r+r/2);
      [center, radius, metric] = imfindcircles(eyesImage2{count+1}, [f1 f2], 'ObjectPolarity','dark',...
                 'Sensitivity', 0.90,'Method','twostage'); % Hough Transform
                
           
      if size(center,1) > 1
         centers2{count+1}=mean(center);
         radii2{count+1}=mean(radius);
      else
         centers2{count+1}=center;
         radii2{count+1}=radius; 
      end  
     
       count=count+1;         
 end
 
end

 
for i=1:length(centers)

  center=centers{i}; 
  
   if isempty(center)
      continue
   end
  
   
  centerX = center(1);
  centerX1 = center(1)+2;
  centerX2 = center(1)-2;
  
 
  figure(4+i)
  subplot(3,2,3)
  imshow(eyesImage{i}); 

   hold on;
viscircles(centers{i}, radii{i},'EdgeColor','b'); % draw a circle on the detect iris  
   
     
  xLimits = get(gca,'XLim');  % Get the range of the x axis
  xLimitsEnd = xLimits(2); % Get the end of x axis
  xLimitsHalf =  xLimitsEnd/2;

  subplot(3,2,4)
  if xLimitsHalf <= centerX1 && xLimitsHalf >= centerX2
                 imshow(straight);
  elseif centerX > xLimitsHalf
                 imshow(right);
 else
                 imshow(left);
             
  end
          
 
  subplot(3,2,5)
  imshow(eyesImage2{i});
  
  hold on;

  viscircles(centers2{i}, radii2{i},'EdgeColor','b'); % draw a circle on the detect iris
   
  center=centers2{i}; 
  centerX = center(1);
  centerX1 = center(1)+2;
  centerX2 = center(1)-2;

  xLimits = get(gca,'XLim');  % Get the range of the x axis
  xLimitsEnd = xLimits(2); % Get the end of x axis
  xLimitsHalf =  xLimitsEnd/2;

  subplot(3,2,6)

  if xLimitsHalf <= centerX1 && xLimitsHalf >= centerX2
         imshow(straight);
       
  elseif centerX > xLimitsHalf
         imshow(right);
  else
         imshow(left);

  end
     
end

 
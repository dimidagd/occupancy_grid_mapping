function [Odo_int]=OdometryEstimator(TimeToFind) 
%% --------------------------D.1---------------------------
% clear all;
text = fileread('/Users/admin/SMR_common_folder/odoPose.log'); %Get satellite data set
lines = string(strsplit(text,'\n'));                                      %Take apart data set to individual lines
numlines = size(lines,2)-1;                                         %Get the number of satellites                                                     %Setting blocksize
%% --------------------------D.4---------------------------
for m=1:numlines
  Splt=strsplit(lines(m),' ');
  XYT=[str2num(char(Splt(2))),str2num(char(Splt(3))),str2num(char(Splt(4)))];
  Odo(m).Time=         str2num(char(Splt(1)));                         %Storing time components individually
  Odo(m).XYT=          XYT;
  Odo(m).I =           str2num(char(Splt(5)));
  Odo(m).II =          str2num(char(Splt(6)));
  Odo(m).III =         str2num(char(Splt(7)));
end

%% --------------------------D.7---------------------------
for i=1:length(TimeToFind)
  difference=[Odo.Time];  
  difference=abs(difference-TimeToFind(i));                                     %by choosing the smallest difference between given and required data points
  [M,I] = min(difference);                                                  %Gives the value and index of the smallest separation for the estimation

  if (I < 2)                                                                %If the interpolation point is within 5 data values to either end of the data segment
      I=2;                                                                  %Use the first or last 11 data points respectively
      if I>(numlines-2)
        I=numlines-2;
      end
  end
  if(I==numlines)
      I=I-1;
  end
  interpIndices=(I-1):(I+1); 
 
  for n=1:length(interpIndices)
      TP(n,:)=[Odo(interpIndices(n)).Time,Odo(interpIndices(n)).XYT];
  end

  Odo_int(i).Time=TimeToFind(i);
  Odo_int(i).X=interp1(TP(:,1),TP(:,2),TimeToFind(i));
  Odo_int(i).Y=interp1(TP(:,1),TP(:,3),TimeToFind(i));
  Odo_int(i).T=interp1(TP(:,1),TP(:,4),TimeToFind(i));
end
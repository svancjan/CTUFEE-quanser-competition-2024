load Qpath.mat
xRef = QPath(:,1);
yRef = QPath(:,2);
plot(xRef,yRef);axis equal
v_ref = ones(size(xRef));
hold on;
% scatter(xRef(200),yRef(200));
% scatter(xRef(370),yRef(370));
% scatter(xRef(730),yRef(730));
% scatter(xRef(870),yRef(870));
% scatter(xRef(1100),yRef(1100));
% scatter(xRef(1290),yRef(1290));
% scatter(xRef(end),yRef(end));
v_ref(1:99) = 3;
v_ref(100:199) = 3.6;
v_ref(200:369) = 1.6;
v_ref(370:729) = 3.6;
v_ref(730:869) = 1.6;
v_ref(870:1099) = 3.6;
v_ref(1100:1289) = 1.6;
v_ref(1290:end-10) = 3.6;
v_ref(end-9:end) = 0;
% up
for i = 1:length(QPath)
    if (QPath(i,2) > 3)
        QPath(i,2) = (QPath(i,2)-3)*0.94+3;
    end
end

% left
for i = 1:length(QPath)
    if (QPath(i,1) < -0.8)
        QPath(i,1) = (QPath(i,1)+0.8)*0.95 - 0.8;
    end
end
% right
for i = 1:length(QPath)
    if (QPath(i,1) > 1)
        QPath(i,1) = (QPath(i,1)- 1)*0.96 +1;
    end
end


hold off;
plot(v_ref)
QPath(:,5) = v_ref;
save("Qpath1.mat","QPath")
scatter(QPath(:,1),QPath(:,2))
axis equal


fileID = fopen('QPath.txt','w');

% Write the array to the file
fprintf(fileID,'%d,%d,%d,%d,%d\n', QPath.');

% Close the file
fclose(fileID);

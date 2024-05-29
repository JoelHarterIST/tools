clear;clc

folderpath='C:\Users\ist\Desktop\harter\GNC-proj-Harter';% path to folder where input data .csv file is stored
infilename='initial_data_summary';% name of input .csv file
outfilename='final_data_summary';% desired name of output .csv file
historytablename='history';% desired base name of history table .csv files
historytablesubfolder='history';% desired name of subfolder where history files will be stored

r_D = Constants.a+230e3;  % [m] desired orbital radius
i_D = 42.2;      % [deg] desired orbital inclination

disp('loading initial state data from table...')
tbl=readtable([folderpath,'\',infilename,'.csv']);

n=size(tbl,1);% number of Monte Carlo runs
i_store=10;% skip value for storing historical data
t=zeros(n,1);
r=zeros(n,3);
v=zeros(n,3);
History=cell(n,1);

for i=1:n
    clc
    disp(['Running Monte Carlo: ',num2str(i/n*100,3),'%'])

    [r(i,:),v(i,:),t(i),History{i}]=PEG_3DOFsim(...
        [tbl.pos_ECI_X(i),tbl.pos_ECI_Y(i),tbl.pos_ECI_Z(i)],...
        [tbl.vel_ECI_X(i),tbl.vel_ECI_Y(i),tbl.vel_ECI_Z(i)],...
        tbl.mass(i),...
        tbl.thrust(i),...
        tbl.Isp(i),...
        r_D,i_D);
    nH=size(History{i},1);% number of entries in historical data for this MC iteration
    I=false(1,nH);I(1:i_store:nH)=true;I(end)=true;
    History{i}=History{i}(I,:);% cull the historical data
end

clc
disp('writing final state table')
writetable(...
    array2table([(0:n-1)',t,r,v],"VariableNames",{'case','t','pos_ECI_X','pos_ECI_Y','pos_ECI_Z','vel_ECI_X','vel_ECI_Y','vel_ECI_Z'}),...
    [folderpath,'\',outfilename,'.csv']);

mkdir([folderpath,'\',historytablesubfolder])
for i=1:n
    clc
    disp(['writing history tables ',num2str(i/n*100,3),'%'])
    writetable(...
        array2table(History{i},"VariableNames",{'t','pos_ECI_X','pos_ECI_Y','pos_ECI_Z','vel_ECI_X','vel_ECI_Y','vel_ECI_Z','thr_ECI_X','thr_ECI_Y','thr_ECI_Z'}),...
        [folderpath,'\',historytablesubfolder,'\',historytablename,'_',num2str(i-1,'%05.f'),'.csv'])
end

clc
disp('finished')
%%
% clf
% vlen=5;% multiplier for velocity vector display
% for i=1:n
%     % ploteze(r_0(i,:)+[0;vlen].*v_0(i,:),'b-')
%     hold on
%     ploteze(r(i,:)+[0;vlen].*v(i,:),'r-')
% end

% ploteze(r_0,'bo')
% hold on
% ploteze(r,'r.')
% ploteze(v,'b.')

% clf
% ploteze(uvec(History{1}(:,2:4))*Constants.a,'b')
% hold on
% ploteze(uvec(History{1}(:,2:4))*r_D,'c')
% for i=1:5
%     ploteze(History{i}(:,2:4),'k')
% end











%%% NOTES:
%Done as undirected graph. would have to change adjacency and env file to make directed..
%Variants of tolman maze inc
%1. goal state location changes
%2. transition revaluation
num_act=4;
map = [
    '###.#####';
    '#.......#';
    '#.#.###.#';
    '#.#.###.#';
    '#.#.###.#';
    '#.......#';
    '###.#####'];
%Build tolman ENV
map = [
    '###$####';
    '#......#';
    '#.#.##.#';
    '#.#.##.#';
    '#.#.##.#';
    '#......#';
    '###X####'];

%Number of states
n_s=size(map,1)*size(map,2);
%Set to 2 on diagonal because that's what Noah had??
correctAdjWeights=zeros(size(map,1)*size(map,2),size(map,1)*size(map,2));
ind=flipud(reshape(1:(size(map,1)*size(map,2)),size(map,2),size(map,1))');
M=zeros(size(ind));

for i=1:size(map,1)
    for j=1:size(map,2)
        M(:) = 0;
        temp=ind(i,j);
        M(i,j) = 1; % location
        
        correctAdjWeights(temp,ind(conv2(M,[0,1,0;1,0,1;0,1,0],'same')>0))=1;
    end
end
%Remove barriers/blocked points
for i=1:size(map,1)
    for j=1:size(map,2)
        temp=ind(i,j);
        if strcmp(map(i,j),'#')
            correctAdjWeights(temp,:)=0;
            correctAdjWeights(:,temp)=0;
        end
    end
end

Transition_Mat=pinv(diag(sum(correctAdjWeights,2)))*correctAdjWeights;
tick=1;

for j=0.1:0.1:0.9
    SR_Mat(:,:,i,tick)=pinv(eye(size(correctAdjWeights))-(j)*Transition_Mat);
    tick=tick+1;
end


%TO S to x A (up,down,left,right) x From S
correctMotorWeights=zeros((size(map,1)*size(map,2)),num_act,(size(map,1)*size(map,2)));

%Flip through each row and then column to find
for i=1:size(correctAdjWeights,1)
    [r,c]=ind2sub([size(map,1) size(map,2)],find(ind==i));
    
    pl=find(correctAdjWeights(i,:));
    for j=1:length(pl)
        [rt,ct]=ind2sub([size(map,1) size(map,2)],find(ind==pl(j)));
        dt=[r-rt,c-ct];
        if dt(1,1)==1 %aBOVE
            act=1;
        elseif dt(1,1)==-1 %Below
            act=2;
        elseif dt(1,2)==1 % Left
            act=3;
        elseif dt(1,2)==-1 %Righti
            act=4;
        end
        correctMotorWeights(pl(j),act,i)=1;
    end
end

CM=correctMotorWeights;
correctMotorWeights=[];
for i=1:size(CM,3)
    correctMotorWeights=[correctMotorWeights; CM(:,:,i)];
end


correctAdjWeights= correctAdjWeights+ 2*eye(size(map,1)*size(map,2),size(map,1)*size(map,2));
correctGradWeights=correctAdjWeights-eye(size(map,1)*size(map,2),size(map,1)*size(map,2));

transToWeights=repmat(eye(size(correctAdjWeights)),1,size(correctAdjWeights,1));
transFromWeights=zeros(size(transToWeights,1),size(transToWeights,1),size(transToWeights,1));


for i=1:size(transFromWeights,1)
    transFromWeights(i,:,i)=ones(1,n_s);
end
transFromWeights=(transFromWeights(:,:));

save('Tolman_A_weights.mat','map','correctAdjWeights','correctGradWeights','correctMotorWeights','transToWeights','transFromWeights')

%Make environment file
Tolman_A=[];

for i=1:n_s
    idx=find(sum(CM(:,:,i),2));
    if ~isempty(idx)
    for j=1:length(idx)
        Tolman_A=[Tolman_A;i,idx(j),find(CM(idx(j),:,i))];
    end
    end
end

save('Tolman_A.mat','Tolman_A')

%%%% NEXT MAP
map{2} = [
    '########$########';
    '########.########';
    '#...............#';
    '#.######.######.#';
    '#.######.######.#';
    '#.######.######.#';
    '#.######.######.#';
    '#.######.######.#';
    '#.######.######.#';
    '#...............#';
    '#####.###########'];

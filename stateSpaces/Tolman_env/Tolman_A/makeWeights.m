load('Tolman_A.mat')
n_s=size(unique([Tolman_A(:,1);Tolman_A(:,2)]),1);
num_act=4;
%% Make adjacency weight matrix
correctAdjWeights=zeros(length(unique([Tolman_A(:,1);Tolman_A(:,2)])),length(unique([Tolman_A(:,1);Tolman_A(:,2)])));
for i=1:length(Tolman_A)
    correctAdjWeights(Tolman_A(i,1),Tolman_A(i,2))=1;
        correctAdjWeights(Tolman_A(i,2),Tolman_A(i,1))=1;

end
    

correctAdjWeights=correctAdjWeights+2*eye(n_s,n_s);
correctGradWeights=correctAdjWeights-eye(n_s,n_s);

transToWeights=repmat(eye(size(correctAdjWeights)),1,size(correctAdjWeights,1));
transFromWeights=zeros(size(transToWeights,1),size(transToWeights,1),size(transToWeights,1));
for i=1:size(transFromWeights,1)
transFromWeights(i,:,i)=ones(1,n_s);
end
transFromWeights=(transFromWeights(:,:));




%TO S to x A (up,down,left,right) x From S
correctMotorWeights=zeros(n_s,num_act,n_s);

for i=1:length(Tolman_A)
    
correctMotorWeights(Tolman_A(i,2),Tolman_A(i,3),Tolman_A(i,1))=1;
end
CM=correctMotorWeights;
correctMotorWeights=[];
for i=1:size(CM,3)
    correctMotorWeights=[correctMotorWeights; CM(:,:,i)];
end

save('Tolman_A_weights.mat','correctAdjWeights','correctGradWeights','correctMotorWeights','transToWeights','transFromWeights')
function [e, n] = create_Tolman_B_Net_core(set_weights,verst)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sets up network and environment
% called by batch script
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global dt

load('stateSpaces/Tolman_env/Tolman_A/Tolman_A_weights.mat');
load('stateSpaces/Tolman_env/Tolman_A/Tolman_A.mat');
e=stateSpace(Tolman_A, 1); %(transitionMatrix, starting location)

numStates =  e.get_numStates();
numActions = e.get_numActions();

n=network();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LAYERS AND NODES
%addLayer(name, {numUnits, actFunc, timeConst, decayRate, traceType(optional)})

%layer.set_inhibitors(nodeName, [v1, v2, dt-sign]); inhibition will
%only occur when node value is between v1 and v2 with the appropriate
%dt sign (0 = positive or negative)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if verst==1
%=============================
%== Choosing the next state ==
%=============================

n.addLayer('state', {numStates, 'shunt', .5, 1});

n.addLayer('state2', {numStates, 'shunt', .5, 1});

n.addLayer('goal', {numStates, 'shunt', 1, 1});

n.addLayer('gradient', {numStates, 'easyInhib', 1, 0});
n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1});
n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});
n.so('adjacent').noiseGain=.1; %for visibility only

n.addLayer('next', {numStates, 'easyInhib', 1, .1});
n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]}});

%=========================
%== Monitoring the past ==
%=========================

n.addLayer('prevState1', {numStates, 'shunt', 4, .5});
n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

%=========================
%== Determining Action ===
%=========================

n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]}});

n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; TC, decay was .2, 2 9/29
n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});

%===========
%== Nodes ==
%===========

%External Inputs
n.addNode('n_env', numStates);
n.setNode('n_env', e.get_locVec);
n.connectNode('n_env', 'state');
n.connectNode('n_env', 'state2');

n.addNode('n_goal', numStates);
n.connectNode('n_goal', 'goal');

%Action
%bgFunc explanation:       linear decay         activation if no units active and any input above threshold
bgFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
n.addNode('n_body', numActions);
n.so('n_body').updateFunc=bgFunc;
n.so('n_body').inputs={n.so('motorOut')};

%Inhibition following a state change
n.addNode('n_novInhib', 1);
n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

%Oscillations (second argument is period)
n.addOscNode('n_subOsc', 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PROJECTIONS
%(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
%gate format = {'name', [val1, val2]} gate open between val1 & val2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%=============================
%== Choosing the next state ==
%=============================

n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1);
n.so('p_state_adjacent').learningParams = [.6, .7, .6];

n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}});

n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 15);
n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.55, 1]}};
n.so('p_gradient_gradient').learningParams = [.98, .85, .98];

n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}});
n.so('p_gradient_next').channels={n.so('adjacent'), .3};

n.connect('next', 'next', 'impose', 0, 2, 'null', 0);
n.connect('goal', 'goal', 'impose', 0, 2, 'null', 0); %ADDED


%=========================
%== Monitoring the past ==
%=========================
n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0);

n.connectNode('n_body', 'prevMotor');
n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0);

n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_prevState2_transObs').set_weights(transFromWeights);

n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_state_transObs').set_weights(transToWeights);

n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

%=========================
%== Determining Action ===
%=========================

n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_state_transDes').set_weights(transFromWeights);

n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_next_transDes').set_weights(transToWeights);

n.connect('transDes', 'transDes', 'impose', 0, 1, 'null', 0);
n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}});

n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20, {{n.so('n_subOsc'), [-.2, -1]}});
n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
n.so('p_transOut_motorIn').learningParams = [.42, .15];

n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});






%=============================
%== Choosing the next state ==
%=============================

%% OR LEARN STATE_REWARD MAPPING. THEN PROJECT THRU ADJACENT_REWARD
  n.addLayer('reward', {numStates, 'shunt', .5, 0.1}) % OK - could change .9 to something lower 
n.so('reward').noiseGain=0.1; % OK
 n.so('reward').set_inhibitors({{'n_novInhib', [.65, 1, 0]}}); % OK - inhibit it when the state changes
 n.connect('state2', 'reward', 'one-to-one', 0, -15, 'null', 1, {{'n_subOsc', [-.2, -1, 0]}}); % OK
 n.connect('prevState1', 'reward', 'one-to-one', 0, -15, 'null', 1, {{'n_subOsc', [-.2, -1, 0]}}); % OK



n.addNode('n_reward', numStates);

n.connectNode('n_reward', 'reward'); %The problem is its always on
n.connect('state', 'reward', 'one-to-one', 0, 2, 'adjacency', 1);
n.so('p_state_reward').learningParams = [.6, .7, .6];

%V_A
%   n.so('p_gradient_next').addMod({'reward',1})  % TESTING

 %V_B
% - 0.4 set here (originally 1) 
%  n.connect('reward','next', 'one-to-one', 0, 0.4, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
%  n.so('p_reward_next').channels={n.so('adjacent'), .3};
% and just have reward mod at some val...

%V_C
 n.connect('reward','gradient', 'one-to-one', 0, 1, 'null', 0, {{'n_subOsc', [-1, -.2, 0]}}); %Try changing (1) weight or (2) impose to something else. (3) las

% CHECK
% 1. reward ,state' mapping is learnt
% 2. check oscillation cycle is proper to acting phase (1) versus learning
% phase (0)






%Set optimal weights loaded from _weights.mat file at top of script
if set_weights
    n.so('p_gradient_gradient').set_weights(correctGradWeights);
    n.so('p_gradient_gradient').learnRate=0;
    
    n.so('p_state_adjacent').set_weights(correctAdjWeights);
    n.so('p_state_adjacent').learnRate=0;
    
    n.so('p_transOut_motorIn').set_weights(correctMotorWeights);
    n.so('p_transOut_motorIn').learnRate=0;
end
elseif verst==2
%=============================
%== Choosing the next state ==
%=============================

n.addLayer('state', {numStates, 'shunt', .5, 1});

n.addLayer('state2', {numStates, 'shunt', .5, 1});

n.addLayer('goal', {numStates, 'shunt', 1, 1});

n.addLayer('gradient', {numStates, 'easyInhib', 1, 0});
n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1});
n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});
n.so('adjacent').noiseGain=.1; %for visibility only

n.addLayer('next', {numStates, 'easyInhib', 1, .1});
n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]}});

%=========================
%== Monitoring the past ==
%=========================

n.addLayer('prevState1', {numStates, 'shunt', 4, .5});
n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

%=========================
%== Determining Action ===
%=========================

n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]}});

n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; TC, decay was .2, 2 9/29
n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});

%===========
%== Nodes ==
%===========

%External Inputs
n.addNode('n_env', numStates);
n.setNode('n_env', e.get_locVec);
n.connectNode('n_env', 'state');
n.connectNode('n_env', 'state2');

n.addNode('n_goal', numStates);
n.connectNode('n_goal', 'goal');

%Action
%bgFunc explanation:       linear decay         activation if no units active and any input above threshold
bgFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
n.addNode('n_body', numActions);
n.so('n_body').updateFunc=bgFunc;
n.so('n_body').inputs={n.so('motorOut')};

%Inhibition following a state change
n.addNode('n_novInhib', 1);
n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

%Oscillations (second argument is period)
n.addOscNode('n_subOsc', 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PROJECTIONS
%(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
%gate format = {'name', [val1, val2]} gate open between val1 & val2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%=============================
%== Choosing the next state ==
%=============================

n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1);
n.so('p_state_adjacent').learningParams = [.6, .7, .6];

n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}});

n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 15);
n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.55, 1]}};
n.so('p_gradient_gradient').learningParams = [.98, .85, .98];

n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}});
n.so('p_gradient_next').channels={n.so('adjacent'), .3};

n.connect('next', 'next', 'impose', 0, 2, 'null', 0);
n.connect('goal', 'goal', 'impose', 0, 2, 'null', 0); %ADDED


%=========================
%== Monitoring the past ==
%=========================
n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0);

n.connectNode('n_body', 'prevMotor');
n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0);

n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_prevState2_transObs').set_weights(transFromWeights);

n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_state_transObs').set_weights(transToWeights);

n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

%=========================
%== Determining Action ===
%=========================

n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_state_transDes').set_weights(transFromWeights);

n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_next_transDes').set_weights(transToWeights);

n.connect('transDes', 'transDes', 'impose', 0, 1, 'null', 0);
n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}});

n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20, {{n.so('n_subOsc'), [-.2, -1]}});
n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
n.so('p_transOut_motorIn').learningParams = [.42, .15];

n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});






%=============================
%== Choosing the next state based on reward==
%=============================

%% OR LEARN STATE_REWARD MAPPING. THEN PROJECT THRU ADJACENT_REWARD
n.addLayer('reward', {numStates, 'shunt', .5, 1})
n.so('reward').noiseGain=0.05;
%Add and connect external reward node
n.addNode('n_reward', numStates);
n.connectNode('n_reward', 'reward');

%smush down activity related to current state == state2 in an oscillation
%phase
%%% -10 here pushes the weights far down and creates''' suppression'''
n.connect('state2', 'reward', 'one-to-one', 0, -10, 'null', 1, {{'n_subOsc', [-.2, -1, 0]}});

%learning rewards adjacent to the current state as weights.
n.connect('state', 'reward', 'one-to-one', 0, 2, 'adjacency', 1);
n.so('p_state_reward').learningParams = [.6, .7, .6];

% Changes here...
 n.so('reward').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});





% - 0.4 set here (originally 1) 
 n.connect('reward','next', 'one-to-one', 0, 0.4, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
 n.so('p_reward_next').channels={n.so('adjacent'), .3};
% n.so('p_gradient_next').addMod('reward')  % COULD REMOVE p_reward_next,
% and just have reward mod at some val...

%  n.connect('reward','gradient', 'impose', 0, 1, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); %Try changing (1) weight or (2) impose to something else


%COULD MAP TO ADJACENT or modulate next or learn a mapping to next????






%Set optimal weights loaded from _weights.mat file at top of script
if set_weights
    n.so('p_gradient_gradient').set_weights(correctGradWeights);
    n.so('p_gradient_gradient').learnRate=0;
    
    n.so('p_state_adjacent').set_weights(correctAdjWeights);
    n.so('p_state_adjacent').learnRate=0;
    
    n.so('p_transOut_motorIn').set_weights(correctMotorWeights);
    n.so('p_transOut_motorIn').learnRate=0;
end

elseif verst==3
 %=============================
%== Choosing the next state ==
%=============================

n.addLayer('state', {numStates, 'shunt', .5, 1});


n.addLayer('goal', {numStates, 'shunt', 1, 1});

n.addLayer('gradient', {numStates, 'easyInhib', 1, 0});
n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1});
n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});
n.so('adjacent').noiseGain=.1; %for visibility only

n.addLayer('next', {numStates, 'easyInhib', 1, .1});
n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]}});

%=========================
%== Monitoring the past ==
%=========================

n.addLayer('prevState1', {numStates, 'shunt', 4, .5});
n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

%=========================
%== Determining Action ===
%=========================

n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]}});

n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}});

n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; TC, decay was .2, 2 9/29
n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});

%===========
%== Nodes ==
%===========

%External Inputs
n.addNode('n_env', numStates);
n.setNode('n_env', e.get_locVec);
n.connectNode('n_env', 'state');

n.addNode('n_goal', numStates);
n.connectNode('n_goal', 'goal');

%Action
%bgFunc explanation:       linear decay         activation if no units active and any input above threshold
bgFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
n.addNode('n_body', numActions);
n.so('n_body').updateFunc=bgFunc;
n.so('n_body').inputs={n.so('motorOut')};

%Inhibition following a state change
n.addNode('n_novInhib', 1);
n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

%Oscillations (second argument is period)
n.addOscNode('n_subOsc', 2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PROJECTIONS
%(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
%gate format = {'name', [val1, val2]} gate open between val1 & val2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%=============================
%== Choosing the next state ==
%=============================

n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1);
n.so('p_state_adjacent').learningParams = [.6, .7, .6];

n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}});

n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 15);
n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.55, 1]}};
n.so('p_gradient_gradient').learningParams = [.98, .85, .98];

n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}});
n.so('p_gradient_next').channels={n.so('adjacent'), .3};

n.connect('next', 'next', 'impose', 0, 2, 'null', 0);
n.connect('goal', 'goal', 'impose', 0, 2, 'null', 0); %ADDED


%=========================
%== Monitoring the past ==
%=========================
n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0);

n.connectNode('n_body', 'prevMotor');
n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0);

n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_prevState2_transObs').set_weights(transFromWeights);

n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
n.so('p_state_transObs').set_weights(transToWeights);

n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

%=========================
%== Determining Action ===
%=========================

n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_state_transDes').set_weights(transFromWeights);

n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0);
n.so('p_next_transDes').set_weights(transToWeights);

n.connect('transDes', 'transDes', 'impose', 0, 1, 'null', 0);
n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}});

n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20, {{n.so('n_subOsc'), [-.2, -1]}});
n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
n.so('p_transOut_motorIn').learningParams = [.42, .15];

n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});




   
    
%Set optimal weights loaded from _weights.mat file at top of script
if set_weights
    n.so('p_gradient_gradient').set_weights(correctGradWeights);
    n.so('p_gradient_gradient').learnRate=0;
    
    n.so('p_state_adjacent').set_weights(correctAdjWeights);
    n.so('p_state_adjacent').learnRate=0;
    
    n.so('p_transOut_motorIn').set_weights(correctMotorWeights);
    n.so('p_transOut_motorIn').learnRate=0;
end

elseif verst==4 %Hanoi space copy
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LAYERS AND NODES
    %addLayer(name, {numUnits, actFunc, timeConst, decayRate, traceType(optional)})
    
    %layer.set_inhibitors(nodeName, [v1, v2, dt-sign]); inhibition will
    %only occur when node value is between v1 and v2 with the appropriate
    %dt sign (0 = positive or negative) 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %=============================
    %== Choosing the next state ==
    %=============================

    n.addLayer('state', {numStates, 'shunt', .5, 1}); 
    n.addLayer('goal', {numStates, 'shunt', 1, 1});

    n.addLayer('gradient', {numStates, 'easyInhib', 1, 0}); 
    n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1}); 
    n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});
    n.so('adjacent').noiseGain=.1; %for visibility only
    
    n.addLayer('next', {numStates, 'easyInhib', 1, .1});
    n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]}});

    %=========================
    %== Monitoring the past ==
    %=========================

    n.addLayer('prevState1', {numStates, 'shunt', 4, .5}); 
    n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
    n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

    n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
    n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

    n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

    %=========================
    %== Determining Action ===
    %=========================
    
    n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]}});

    n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; TC, decay was .2, 2 9/29
    n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

    n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});

    %===========
    %== Nodes ==
    %===========

    %External Inputs
    n.addNode('n_env', numStates);
    n.setNode('n_env', e.get_locVec);
    n.connectNode('n_env', 'state');

    n.addNode('n_goal', numStates);
    n.connectNode('n_goal', 'goal'); 

    %Action 
    %bgFunc explanation:       linear decay         activation if no units active and any input above threshold
    bgFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
    n.addNode('n_body', numActions);
    n.so('n_body').updateFunc=bgFunc;
    n.so('n_body').inputs={n.so('motorOut')};

    %Inhibition following a state change
    n.addNode('n_novInhib', 1);
    n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

    %Oscillations (second argument is period)
    n.addOscNode('n_subOsc', 2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PROJECTIONS
    %(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
    %gate format = {'name', [val1, val2]} gate open between val1 & val2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %=============================
    %== Choosing the next state ==
    %=============================

    n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1);
    n.so('p_state_adjacent').learningParams = [.6, .7, .6];
    
    n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

    n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}});  

    n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 15); 
    n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.55, 1]}};
    n.so('p_gradient_gradient').learningParams = [.98, .85, .98];

    n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
    n.so('p_gradient_next').channels={n.so('adjacent'), .3};

    n.connect('next', 'next', 'impose', 0, 2, 'null', 0);


    %=========================
    %== Monitoring the past ==
    %=========================
    n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0); 

    n.connectNode('n_body', 'prevMotor');
    n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0); 

    n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_prevState2_transObs').set_weights(transFromWeights);
    
    n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_state_transObs').set_weights(transToWeights);

    n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
    n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}}); 
    
    n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}});

    %=========================
    %== Determining Action ===
    %=========================

    n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0);    
    n.so('p_state_transDes').set_weights(transFromWeights);

    n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0); 
    n.so('p_next_transDes').set_weights(transToWeights);

    n.connect('transDes', 'transDes', 'impose', 0, 1, 'null', 0);
    n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}});

    n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20, {{n.so('n_subOsc'), [-.2, -1]}});
    n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_transOut_motorIn').learningParams = [.42, .15];

    n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});

    %Set optimal weights loaded from _weights.mat file at top of script
    if set_weights
        n.so('p_gradient_gradient').set_weights(correctGradWeights);
        n.so('p_gradient_gradient').learnRate=0;

        n.so('p_state_adjacent').set_weights(correctAdjWeights);
        n.so('p_state_gradient').learnRate=0;
     
        n.so('p_transOut_motorIn').set_weights(correctMotorWeights);
        n.so('p_transOut_motorIn').learnRate=0;
    end  
    
elseif verst==5  %% Smallenv Copy
     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %LAYERS AND NODES
    %addLayer(name, {numUnits, actFunc, timeConst, decayRate, traceType(optional)})
    
    %layer.set_inhibitors(nodeName, [v1, v2, dt-sign]); inhibition will
    %only occur when node value is between v1 and v2 with the appropriate
    %dt sign (0 = positive or negative) 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %=============================
    %== Choosing the next state ==
    %=============================

    n.addLayer('state', {numStates, 'shunt', .5, 1}); 
    n.addLayer('goal', {numStates, 'shunt', 1, 1});

    n.addLayer('gradient', {numStates, 'easyInhib', 1, 1}); 
    n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1}); 
    n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]}});
    n.so('adjacent').noiseGain=.05; %for visibility only
    
    n.addLayer('next', {numStates, 'easyInhib', 1, .1});
    n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]}});

    %=========================
    %== Monitoring the past ==
    %=========================

    n.addLayer('prevState1', {numStates, 'shunt', 4, .5}); 
    n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
    n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

    n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
    n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

    n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

    %=========================
    %== Determining Action ===
    %=========================
    
    n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]}});

    n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); 
    n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

    n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});

    %===========
    %== Nodes ==
    %===========

    %External Inputs
    n.addNode('n_env', numStates);
    n.setNode('n_env', e.get_locVec);
    n.connectNode('n_env', 'state');

    n.addNode('n_goal', numStates);
    n.connectNode('n_goal', 'goal'); 

    %Action 
    %bgFunc explanation:       linear decay         activation if no units active and any input above threshold
    bgFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
    n.addNode('n_body', numActions);
    n.so('n_body').updateFunc=bgFunc;
    n.so('n_body').inputs={n.so('motorOut')};

    %Inhibition following a state change
    n.addNode('n_novInhib', 1);
    n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

    %Oscillations (second argument is period)
    n.addOscNode('n_subOsc', 2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PROJECTIONS
    %(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
    %gate format = {'name', [val1, val2]} gate open between val1 & val2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %=============================
    %== Choosing the next state ==
    %=============================

    n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1);
    n.so('p_state_adjacent').learningParams = [.6, .7, .6];
    
    n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

    n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}});  

    n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 10); 
    n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_gradient_gradient').learningParams = [.75, .7, .75];

    n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
    n.so('p_gradient_next').channels={n.so('adjacent'), .3};

    n.connect('next', 'next', 'impose', 0, 2, 'null', 0);


    %=========================
    %== Monitoring the past ==
    %=========================
    n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0); 

    n.connectNode('n_body', 'prevMotor');
    n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0); 

    n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_prevState2_transObs').set_weights(transFromWeights);
    
    n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_state_transObs').set_weights(transToWeights);

    n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
    n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]}}); 
    
    n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 10, 'null', 0, {{'n_subOsc', [.2, 1]}});


    %=========================
    %== Determining Action ==
    %=========================

    n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0);    
    n.so('p_state_transDes').set_weights(transFromWeights);

    n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0); 
    n.so('p_next_transDes').set_weights(transToWeights);

    n.connect('transDes', 'transDes', 'impose', 0, 2, 'null', 0);
    n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}});

    n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20);
    n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_transOut_motorIn').learningParams = [.42, .15];

    n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});

    %Set optimal weights loaded from _weights.mat file at top of script
    if set_weights
        n.so('p_gradient_gradient').set_weights(correctGradWeights);
        n.so('p_gradient_gradient').learnRate=0;

        n.so('p_state_adjacent').set_weights(correctAdjWeights);
        n.so('p_state_gradient').learnRate=0;
     
        n.so('p_transOut_motorIn').set_weights(correctMotorWeights);
        n.so('p_transOut_motorIn').learnRate=0;
    end  
elseif verst==6  %Qeue layer small env
    n.addLayer('state', {numStates, 'shunt', .5, 1}); 
    n.addLayer('goal', {numStates, 'shunt', 1, 1});

    n.addLayer('gradient', {numStates, 'easyInhib', 1, 1}); 
    n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1}); 
    n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]},{'n_qOsc',[-.9,-.3,1]}});
    n.so('adjacent').noiseGain=.05; %for visibility only
    
    n.addLayer('next', {numStates, 'easyInhib', 1, .1});
    n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]},{'n_qOsc',[-.4,.1,1]}});

    %=========================
    %== Monitoring the past ==
    %=========================

    n.addLayer('prevState1', {numStates, 'shunt', 4, .5}); 
    n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
    n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

    n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
    n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

    n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

    %=========================
    %== Determining Action ===
    %=========================
    
    n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]},{'n_qOsc',[.1, 1, 1]},{'n_qOsc',[1, .65, -1]}});

    n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}, {'n_qOsc', [-.2,.2,1]}});

    n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; 
    n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

    n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});
    
    %=====================
    %== Storing a Plan ===
    %=====================
    n.addLayer('stateSim',{numStates,'easyInhib',.7,.01});
    n.so('stateSim').set_inhibitors({{'n_novInhib',[.6,1,-1],-5},{'n_qOsc',[-.7,-1,-1]}});
     
    
    n.addLayer('qIn',{numStates^2, 'easyInhib', .2,1});
    n.so('qIn').set_inhibitors({{'n_qOsc',[.05, .15, 1]}}); %[-.6, -.7, -1]}});
    
    n.addLayer('qStore',{numStates^2, 'easyInhib', 1,0});
    
    n.addLayer('qOut', {numStates^2, 'easyInhib', .2,0});
    n.so('qOut').set_inhibitors({{'n_qOsc', [-.2,.2,1]}});
    
    n.addNode('n_qOn',1); % 0=no cue, 1=storing plan, 2=reading plan    
    n.addOscNode('n_qOsc',1.5);
    
    %===========
    %== Nodes ==
    %===========

    %External Inputs
    n.addNode('n_env', numStates);
    n.setNode('n_env', e.get_locVec);
    n.connectNode('n_env', 'state');

    n.addNode('n_goal', numStates);
    n.connectNode('n_goal', 'goal'); 

    %Action 
    %bodyFunc explanation:       linear decay         activation if no units active and any input above threshold
%     bodyFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
    bodyFunc = @(vals, in) (any(vals>0)*max((vals-(dt/3)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));

    
    n.addNode('n_body', numActions);
    n.so('n_body').updateFunc=bodyFunc;
    n.so('n_body').inputs={n.so('motorOut')};

    %Inhibition following a state change
    n.addNode('n_novInhib', 1);
    n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

    %Oscillations (second argument is period)
    n.addOscNode('n_subOsc', 2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PROJECTIONS
    %(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
    %gate format = {'name', [val1, val2]} gate open between val1 & val2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %=============================
    %== Choosing the next state ==
    %=============================

    n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1, {{'n_qOn',[0 0]}});
    n.so('p_state_adjacent').learningParams = [.6, .7, .6];
    
    n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

    n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}}); 

    n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 10); 
    n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_gradient_gradient').learningParams = [.75, .7, .75];

    n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
    n.so('p_gradient_next').channels={n.so('adjacent'), .3};

    n.connect('next', 'next', 'impose', 0, 2, 'null', 0);


    %=========================
    %== Monitoring the past ==
    %=========================
    n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0); 

    n.connectNode('n_body', 'prevMotor');
    n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0); 

    n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_prevState2_transObs').set_weights(transFromWeights);
    
    n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_state_transObs').set_weights(transToWeights);

    n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
    n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]},{'n_qOn', [0,0]}}); 
    
    n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 10, 'null', 0, {{'n_subOsc', [.2, 1]}});


    %=========================
    %== Determining Action ==
    %=========================

    n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0, {{'n_qOn', 0}});    
    n.so('p_state_transDes').set_weights(transFromWeights);

    n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0); 
    n.so('p_next_transDes').set_weights(transToWeights);

    n.connect('transDes', 'transDes', 'impose', 0, 2, 'null', 0);
    n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}, {'n_qOn', 0}});

    n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20);
    n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_transOut_motorIn').learningParams = [.42, .15];

    n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});

    %=====================
    %== Storing a Plan ===
    %=====================
    n.connect('state','stateSim','one-to-one',0,1, 'null', 1, {{'n_qOn',0}}); %state projects to stateSim when queue off

    n.connect('next','stateSim', 'one-to-one',0, 6,'null',0,{{'n_qOn',[.99 1.01]},{'n_qOsc',[-.9,-.6,1]}});

    n.connect('stateSim', 'stateSim', 'impose', 0, 1, 'null', 1);
    n.connect('stateSim', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1, {{'n_qOn', 1}});
    n.so('p_stateSim_adjacent').learnGate={{'n_qOn',[0 0]}};


    n.connect('stateSim', 'transDes', 'unif', 0, 1, 'null', 0, {{'n_qOn', 1}});
    n.so('p_stateSim_transDes').set_weights(transFromWeights);

    n.connect('transDes', 'qIn', 'one-to-one', 0, 8, 'null', 1,{{'n_qOn', 1},{'n_qOsc',[-.2,0,1]}});
    n.connect('qIn', 'qStore', 'one-to-one', 0, 12, 'null', 1);
    n.connect('qStore', 'qIn', 'all-to-all', 0, -2.5, 'null', 1);
    
    n.connect('qStore','qOut','one-to-one',0,2,'null',1, {{'n_qOn', 2}, {'n_qOsc', [.8,1,0]}});
    n.connect('qOut','qOut','impose',0,2,'null',0);
    
    n.connect('qOut','qStore','one-to-one',0,-5,'null',1,{{'n_qOsc', [-.7,-1,0]}});
    
    n.connect('qOut','transOut','one-to-one',0,3,'null',1);
    
elseif verst==7 %queue layer hanoi equiv env
     n.addLayer('state', {numStates, 'shunt', .5, 1}); 
    n.addLayer('goal', {numStates, 'shunt', 1, 1});

    n.addLayer('gradient', {numStates, 'easyInhib', 1, 0}); 
    n.so('gradient').set_inhibitors({{'n_novInhib', [.8, 1, 0], -8}, {'n_subOsc', [-.2, .2, 0]}});

    n.addLayer('adjacent', {numStates, 'easyInhib', .2, 1}); 
    n.so('adjacent').set_inhibitors({{'n_novInhib', [.65, 1, 0]},{'n_qOsc',[-.9,-.3,1]}});
    n.so('adjacent').noiseGain=.05; %for visibility only
    
    n.addLayer('next', {numStates, 'easyInhib', 1, .1});
    n.so('next').set_inhibitors({{'n_novInhib', [.5, .05, 0]},{'n_qOsc',[-.4,.1,1]}});

    %=========================
    %== Monitoring the past ==
    %=========================

    n.addLayer('prevState1', {numStates, 'shunt', 4, .5}); 
    n.addLayer('prevState2', {numStates, 'easyInhib', 1, .001});
    n.so('prevState2').set_inhibitors({{'n_novInhib', [.7, 1, 0]}});

    n.addLayer('prevMotor', {numActions, 'easyInhib', 1, .001});
    n.so('prevMotor').set_inhibitors({{'n_novInhib', [.55, 1, 0]}});

    n.addLayer('transObs', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transObs').set_inhibitors({{'n_novInhib', [.5, .8, 0]}});

    %=========================
    %== Determining Action ===
    %=========================
    
    n.addLayer('transDes', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transDes').set_inhibitors({{'n_novInhib', [.1, 1, 0]},{'n_qOsc',[.1, 1, 1]},{'n_qOsc',[1, .65, -1]}});

    n.addLayer('transOut', {numStates^2, 'easyInhib', 1.5, 1});
    n.so('transOut').set_inhibitors({{'n_subOsc', [.1, .8, 1]}, {'n_subOsc', [-.2, .2, 0]}, {'n_qOsc', [-.2,.2,1]}});

    n.addLayer('motorIn', {numActions, 'easyInhib', 1, 1}); %6 actions: left->mid, left->right, mid->left, etc.; 
    n.so('motorIn').set_inhibitors({{'n_subOsc', [-.5, .5, 0]}});

    n.addLayer('motorOut', {numActions, 'easyInhib', .5, .2});
    
    %=====================
    %== Storing a Plan ===
    %=====================
    n.addLayer('stateSim',{numStates,'easyInhib',.7,.01});
    n.so('stateSim').set_inhibitors({{'n_novInhib',[.6,1,-1],-5},{'n_qOsc',[-.7,-1,-1]}});
     
    
    n.addLayer('qIn',{numStates^2, 'easyInhib', .2,1});
    n.so('qIn').set_inhibitors({{'n_qOsc',[.05, .15, 1]}}); %[-.6, -.7, -1]}});
    
    n.addLayer('qStore',{numStates^2, 'easyInhib', 1,0});
    
    n.addLayer('qOut', {numStates^2, 'easyInhib', .2,0});
    n.so('qOut').set_inhibitors({{'n_qOsc', [-.2,.2,1]}});
    
    n.addNode('n_qOn',1); % 0=no cue, 1=storing plan, 2=reading plan    
    n.addOscNode('n_qOsc',1.5);
    
    %===========
    %== Nodes ==
    %===========

    %External Inputs
    n.addNode('n_env', numStates);
    n.setNode('n_env', e.get_locVec);
    n.connectNode('n_env', 'state');

    n.addNode('n_goal', numStates);
    n.connectNode('n_goal', 'goal'); 

    %Action 
    %bodyFunc explanation:       linear decay         activation if no units active and any input above threshold
%     bodyFunc = @(vals, in) (any(vals>0)*max((vals-(dt/5)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));
    bodyFunc = @(vals, in) (any(vals>0)*max((vals-(dt/3)), 0))+(all(vals<=0).*(in>=.6).*(in==max(in)));

    
    n.addNode('n_body', numActions);
    n.so('n_body').updateFunc=bodyFunc;
    n.so('n_body').inputs={n.so('motorOut')};

    %Inhibition following a state change
    n.addNode('n_novInhib', 1);
    n.so('n_novInhib').updateFunc = @(x) max(x - (dt/2), 0);

    %Oscillations (second argument is period)
    n.addOscNode('n_subOsc', 2);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %PROJECTIONS
    %(obj, sourceName, targetName, weightConfig, norm, weightFactor, learnType, learnRate, gates)
    %gate format = {'name', [val1, val2]} gate open between val1 & val2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %=============================
    %== Choosing the next state ==
    %=============================

    n.connect('state', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1, {{'n_qOn',[0 0]}});
    n.so('p_state_adjacent').learningParams = [.6, .7, .6];
    
    n.connect('state', 'gradient', 'one-to-one', 0, 4, 'null', 1, {{'n_subOsc', [.2, 1, 0]}});

    n.connect('goal', 'gradient', 'one-to-one', 0, 4, 'null', 0, {{'n_subOsc', [-.2, -1, 0]}}); 

    n.connect('gradient', 'gradient', 'one-to-one', 0, 1, 'gradient', 10); 
    n.so('p_gradient_gradient').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_gradient_gradient').learningParams = [.75, .7, .75];

    n.connect('gradient', 'next', 'one-to-one', 0, 2, 'null', 0, {{'n_subOsc', [-1, -.2, 1]}}); 
    n.so('p_gradient_next').channels={n.so('adjacent'), .3};

    n.connect('next', 'next', 'impose', 0, 2, 'null', 0);


    %=========================
    %== Monitoring the past ==
    %=========================
    n.connect('state', 'prevState1', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState1', 'prevState2', 'one-to-one', 0, 1, 'null', 0);
    n.connect('prevState2', 'prevState2', 'imposeStrong', 0, 1, 'null', 0); 

    n.connectNode('n_body', 'prevMotor');
    n.connect('prevMotor', 'prevMotor', 'imposeStrong', 0, 2, 'null', 0); 

    n.connect('prevState2', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_prevState2_transObs').set_weights(transFromWeights);
    
    n.connect('state', 'transObs', 'unif', 0, 1, 'null', 0);
    n.so('p_state_transObs').set_weights(transToWeights);

    n.connect('transObs', 'transObs', 'impose', 0, 1, 'null', 0);
    n.connect('transObs', 'transOut', 'one-to-one', 0, 3, 'null', 0, {{'n_subOsc', [.2, 1]},{'n_qOn', [0,0]}}); 
    
    n.connect('prevMotor', 'motorIn', 'one-to-one', 0, 10, 'null', 0, {{'n_subOsc', [.2, 1]}});


    %=========================
    %== Determining Action ==
    %=========================

    n.connect('state', 'transDes', 'unif', 0, 1, 'null', 0, {{'n_qOn', 0}});    
    n.so('p_state_transDes').set_weights(transFromWeights);

    n.connect('next', 'transDes', 'unif', 0, 1, 'null', 0); 
    n.so('p_next_transDes').set_weights(transToWeights);

    n.connect('transDes', 'transDes', 'impose', 0, 2, 'null', 0);
    n.connect('transDes', 'transOut', 'imposeStrong', 0, 1, 'null', 0, {{'n_subOsc', [-.2, -1]}, {'n_qOn', 0}});

    n.connect('transOut', 'motorIn', 'zeros', 0, 1, 'oscHebb', 20);
    n.so('p_transOut_motorIn').learnGate={{n.so('n_subOsc'), [.2, 1]}};
    n.so('p_transOut_motorIn').learningParams = [.42, .15];

    n.connect('motorIn', 'motorOut', 'impose', 0, 6, 'null', 0, {{'n_subOsc', [-.4, -1]}});

    %=====================
    %== Storing a Plan ===
    %=====================
    n.connect('state','stateSim','one-to-one',0,1, 'null', 1, {{'n_qOn',0}}); %state projects to stateSim when queue off

    n.connect('next','stateSim', 'one-to-one',0, 6,'null',0,{{'n_qOn',[.99 1.01]},{'n_qOsc',[-.9,-.6,1]}});

    n.connect('stateSim', 'stateSim', 'impose', 0, 1, 'null', 1);
    n.connect('stateSim', 'adjacent', 'one-to-one', 0, 2, 'adjacency', 1, {{'n_qOn', 1}});
    n.so('p_stateSim_adjacent').learnGate={{'n_qOn',[0 0]}};


    n.connect('stateSim', 'transDes', 'unif', 0, 1, 'null', 0, {{'n_qOn', 1}});
    n.so('p_stateSim_transDes').set_weights(transFromWeights);

    n.connect('transDes', 'qIn', 'one-to-one', 0, 8, 'null', 1,{{'n_qOn', 1},{'n_qOsc',[-.2,0,1]}});
    n.connect('qIn', 'qStore', 'one-to-one', 0, 12, 'null', 1);
    n.connect('qStore', 'qIn', 'all-to-all', 0, -2.5, 'null', 1);
    
    n.connect('qStore','qOut','one-to-one',0,2,'null',1, {{'n_qOn', 2}, {'n_qOsc', [.8,1,0]}});
    n.connect('qOut','qOut','impose',0,2,'null',0);
    
    n.connect('qOut','qStore','one-to-one',0,-5,'null',1,{{'n_qOsc', [-.7,-1,0]}});
    
    n.connect('qOut','transOut','one-to-one',0,3,'null',1);
end
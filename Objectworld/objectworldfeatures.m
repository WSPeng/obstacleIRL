% Construct the features and reward function for the objectworld domain.
function [reward,features_pt,features_dyn] = objectworldfeatures(mdp_params,mdp_data)

POSITIVE = 10;
NEGATIVE = -15;

if mdp_params.rbf_features == 1
    % First create the features, which for now are RBF functions centered at
    % each object.
    features_pt = cell(1,length(mdp_data.objects));
    features_dyn = cell(1,1);
    % features_dyn{1} = struct('type','dist','r',-1.0,'idx',1:mdp_data.udims);
    features_dyn{1} = struct('type', 'hard','r',-1000000.0,'idx',1:mdp_data.udims);
    % features_dyn = {};
    for i=1:length(mdp_data.objects)
        features_pt{i} = struct('type','cartrbf','pos',[mdp_data.objects(i).pos(:,1) mdp_data.objects(i).pos(:,end)],...
                          'width',mdp_params.feature_radius,'r',1.0);
    end

    % Create reward as linear combination of features. Objects where c1 = 1 are
    % assigned a reward of POSITIVE, objects where c1 = 2 are assigned NEGATIVE.
    % First create the weights.
    theta = zeros(1,length(mdp_data.objects)+1);
    theta(1) = mdp_params.step_cost;
    % theta = zeros(1,length(mdp_data.objects));
    for i=1:length(mdp_data.objects)
        if mdp_data.objects(i).c1 == 1
            theta(i+1) = POSITIVE;
            %theta(i) = POSITIVE;
        elseif mdp_data.objects(i).c1 == 2
            theta(i+1) = NEGATIVE;
            %theta(i) = NEGATIVE;
            %{
        elseif mdp_data.objects(i).c1 == 3
            theta(i+1) = POSITIVE*2.0;
            features_pt{i}.width = features_pt{i}.width/2;
        elseif mdp_data.objects(i).c1 == 4
            theta(i+1) = POSITIVE*0.7;
        elseif mdp_data.objects(i).c1 == 5
            theta(i+1) = POSITIVE*1.5;
            features_pt{i}.width = features_pt{i}.width/2;
            %}
        end
    end
elseif mdp_params.rbf_features == 2
    % 1 rbf and one dimensional reward
    features_pt = cell(1,1);
    features_dyn = cell(1,1);
    % features_dyn{1} = struct('type','dist','r',-1.0,'idx',1:mdp_data.udims);
    % features_dyn{1} = struct('type', 'hard','r',-10000000.0,'idx',1:mdp_data.udims);
    % features_dyn{1} = struct('type', 'hard','r',-10000000.0,'idx',1:mdp_data.udims);
    features_dyn{1} = struct('type', 'hardcube','r',-1.0,'idx',1:mdp_data.udims);
    features_pt{1} = struct('type','cartrbf','pos',[mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                          'width',0.05/mdp_data.bounds(1),'r',2.0);
    features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
        'width',0.05/mdp_data.bounds(1),'r',2.0);
    % features_dyn{1} = struct('type','cartrbf','pos',[0.5, 1.0],'width',1.0,'r',2.0);
                      % features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
    %                       'width',1.0,'r',2.0);
    % features_pt{3} = struct('type','cartrbf','pos',[mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
    %                       'width',1.0,'r',2.0);
    % removed the one dimensional reward ...
    % features_pt{length(features_pt)+1} = struct('type','onedim','direction','x','r',1.0,'width',0.05);
    % features_pt{length(features_pt)+1} = struct('type','onedim','direction','y','r',1.0,'width',0.9);
     
    theta = zeros(1,length(features_pt)+1);
    theta(1) = mdp_params.step_cost; % dist
    for i=1:length(mdp_data.objects)
        if mdp_data.objects(i).c1 == 1
            theta(i+1) = POSITIVE;
        elseif mdp_data.objects(i).c1 == 2
            theta(i+1) = NEGATIVE;
        else
            theta(i+1) = mdp_data.objects(i).c1;
        end
    end
else
    % For each object type, create a seperate sum feature with RBF functions
    % centered at each object type.
    features_pt = cell(1,mdp_params.c1*mdp_params.c2);
    features_dyn = cell(1,1);
    features_dyn{1} = struct('type','dist','r',-1.0);
    theta = zeros(1,length(features_pt)+length(features_dyn));
    theta(1) = mdp_params.step_cost;
    empty_features = [];

    % Create colors.
    shapeColors = lines(mdp_params.c1+mdp_params.c2);
    
    % Dump c1 and c2 into arrays.
    c1s = zeros(1,length(mdp_data.objects));
    c2s = zeros(1,length(mdp_data.objects));
    idxs = zeros(1,length(mdp_data.objects));
    for i=1:length(mdp_data.objects)
        c1s(i) = mdp_data.objects(i).c1;
        c2s(i) = mdp_data.objects(i).c2;
        idxs(i) = (c1s(i)-1)*mdp_params.c2 + c2s(i);
    end

    % Create RBFs for each pair of object IDs.
    for c1=1:mdp_params.c1
        for c2=1:mdp_params.c2
            % Compute index.
            idx = (c1-1)*mdp_params.c2 + c2;

            % Count number of matching objects.
            objs = find(idxs == idx);

            % Check for empty objects list.
            if ~isempty(objs)
                % Create rbfs.
                rbfs = cell(1,length(objs));
                for i=1:length(objs)
                    rbfs{i} = struct('type','cartrbf','pos',[mdp_data.objects(objs(i)).pos(:,1) mdp_data.objects(objs(i)).pos(:,end)],...
                                     'width',mdp_params.feature_radius,'r',1.0);
                end

                % Choose colors.
                color1 = shapeColors(c1,:);
                color2 = shapeColors(mdp_params.c1+c2,:);
                
                % Create the sum feature.
                features_pt{idx} = struct('type','sum','theta',ones(1,length(objs)),'features',{rbfs},'color1',color1,'color2',color2);

                % Set the weight on this feature.
                if c1 == 1
                    theta(idx+length(features_dyn)) = POSITIVE;
                elseif c1 == 2
                    theta(idx+length(features_dyn)) = NEGATIVE;
                end
            else
                features_pt{idx} = [];
                empty_features = [empty_features idx];
            end
        end
    end
    
    % Remove the empty features.
    features_pt(empty_features) = [];
    theta(empty_features + length(features_dyn)) = [];
end

% Create the reward.
reward = struct('type','sum','theta',theta,'features',{[features_dyn features_pt]});
% reward = struct('type','sum','theta',theta,'features',{features_pt});

% Now create seperate features if desired.
if strcmp(mdp_params.feature_type,'simple')
    % Simple features - simply identity mappings for the states.
    features_pt = cell(1,mdp_data.dims);
    for i=1:mdp_data.dims
        features_pt{i} = struct('type','id','idx',i,'r',1.0);
    end
elseif strcmp(mdp_params.feature_type,'cartesian')
    % Simple features corresponding to Cartesian coordinates.
    features_pt = cell(1,2);
    features_pt{1} = struct('type','id','idx',1,'r',1.0);
    features_pt{2} = struct('type','id','idx',mdp_data.dims,'r',1.0);
elseif strcmp(mdp_params.feature_type,'grid')
    % RBF grid.
    % redo the feature_pt, the previous one is stored..
    % modified the pos of feature_pt, also the size
    %{
    GRID_STEPS = mdp_params.grid_feature_steps;
    OFFSET = mdp_params.grid_feature_start;
    STEP_SIZE = mdp_params.grid_feature_step;
    features_pt = cell(1,GRID_STEPS*GRID_STEPS);
    for x=1:GRID_STEPS
        for y=1:GRID_STEPS
            i = (y-1)*GRID_STEPS+x;
            pos = [x-1 y-1]*mdp_params.size*STEP_SIZE + [OFFSET OFFSET]*mdp_params.size;
            features_pt{i} = struct('type','cartrbf','pos',pos,...
                              'width',mdp_params.feature_radius,'r',1.0);
        end
    end
    %}
    features_pt = cell(1,3);
    features_pt{1} = struct('type','cartrbf','pos',[0.5 0.22]*mdp_params.size,...
                              'width',mdp_params.feature_radius,'r',1.0);
    features_pt{2} = struct('type','cartrbf','pos',[0 0.22]*mdp_params.size,...was 0.1 0.5
                              'width',mdp_params.feature_radius,'r',1.0);    
    features_pt{3} = struct('type','cartrbf','pos',[1 0.5]*mdp_params.size,...
                              'width',mdp_params.feature_radius,'r',1.0);                          
elseif strcmp(mdp_params.feature_type,'obs')
    % obstacle feature
    features_pt = cell(1,1);
    features_pt{1} = struct('type','cartrbf','pos',[mdp_data.objects(1).pos(:,1), mdp_data.objects(1).pos(:,end)],...
                          'width',0.05/mdp_data.bounds(1),'r',2.0);
    features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
                          'width',0.05/mdp_data.bounds(1),'r',2.0);
    % features_pt{2} = struct('type','cartrbf','pos',[mdp_data.objects(2).pos(:,1), mdp_data.objects(2).pos(:,end)],...
    %                      'width',2.0,'r',2.0);
    %features_pt{3} = struct('type','cartrbf','pos',[mdp_data.objects(3).pos(:,1), mdp_data.objects(3).pos(:,end)],...
    %                      'width',2.0,'r',2.0);
    
    % removed the one dimensional reward ...
%     features_pt{length(features_pt)+1} = struct('type','onedim','direction','x','r',1.0,'width',0.05);
%     features_pt{length(features_pt)+1} = struct('type','onedim','direction','y','r',1.0,'width',0.5);
end

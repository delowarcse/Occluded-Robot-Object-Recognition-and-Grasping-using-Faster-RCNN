
% Create CaoEngine object
cao = actxserver('CAO.CaoEngine');

% Get Workespaces object
ws = cao.Workspaces.Item(int32(0));

% Create Controller object
ctrl = ws.AddController('RC8', 'CaoProv.DENSO.RC8', '', 'Server=192.168.0.1');

% Create Robot Control
global rob;
rob = ctrl.AddRobot('arm');

% takearm
rob.Execute('Motor', [1,0]);
rob.Execute('takearm');
% Set Speed
rob.Speed(-1, 20);
rob.Execute('ExtSpeed',[30,50,50]);
%�d���n���h���[�^
% Create Robot Control
global caoExt;
caoExt = ctrl.AddExtension('Hand0');

% hand
caoExt.Execute('Motor', 1);

State = caoExt.Execute('get_BusyState');
while State~=0
    State = caoExt.Execute('get_BusyState');%�`���b�N�̓���`�F�b�N
end

state_org = caoExt.Execute('get_OrgState');
if state_org==0
    caoExt.Execute('Org');       %���_���A
    State = caoExt.Execute('get_BusyState');
    while State~=0
        State = caoExt.Execute('get_BusyState');
    end
end

%  P1='(440,-230,200,-180,0,180,5)';
% caoExt.Execute('UnChunck',2);
P1='(155,0,300,-180,0,180,5)';
rob.Move(1,P1);
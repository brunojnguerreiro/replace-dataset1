function QuadPlot(pos,ang,auto,Colors,rot_axis,diameter)
% QUADPLOT Plots a quadrotor helicopter from position and orientation
%
%   syntax: QuadPlot(pos,ang,auto,Colors,rot_axis,diameter)
%
%   pos			--> ENU position vector [x;y;z]
%   ang			--> ZYX euler angles vector [phi;th;psi]
%   auto		--> plots heli in the active figure or in an independent one
%					0 -> assuming that "hold" is on
%					1 -> erases active figure and plots the helicopter
%	Colors	    --> colors to use (default: [0.95 0.95 0.0;0.3 0.3 0.3])
%   rot_axis	--> custom rotation for axis corrections (default: eye(3))
%   diameter	--> drone diameter (default: 1)
%
%   examples: QuadPlot([1;2;3],[0;pi/6;pi/12],1); grid on; axis equal;
%
%   Bruno Guerreiro (2015-04-29) (info at https://brunojnguerreiro.eu)
%--------------------------------------------------------------------------

    if ~exist('Colors','var') || isempty(Colors)
        Colors = [0.95 0.95 0.0;0.3 0.3 0.3];
    end

    if ~exist('rot_axis','var') || isempty(rot_axis)
        rot_axis = eye(3);
    end
    internal_rot_axis = [0,1,0;1,0,0;0,0,-1]; % ENU
%     internal_rot_axis = eye(3); % NED
    pos = pos';
    ang = ang';

    if ~exist('diameter','var') || isempty(diameter)
        diameter = 0.6;
    end
    esc = diameter/0.6;
    
    rot = [ cos(ang(3))*cos(ang(2)) , ( cos(ang(3))*sin(ang(2))*sin(ang(1)) - sin(ang(3))*cos(ang(1)) ) , ( cos(ang(3))*sin(ang(2))*cos(ang(1)) + sin(ang(3))*sin(ang(1)) )
            sin(ang(3))*cos(ang(2)) , ( sin(ang(3))*sin(ang(2))*sin(ang(1)) + cos(ang(3))*cos(ang(1)) ) , ( sin(ang(3))*sin(ang(2))*cos(ang(1)) - cos(ang(3))*sin(ang(1)) )
            -sin(ang(2))            , cos(ang(2))*sin(ang(1))                                           , cos(ang(2))*cos(ang(1))                                           ];
    rot = rot_axis*rot*internal_rot_axis;
    
    %--- arm
    n_arms = 4;
    for i = 1:n_arms
        ang = 2*pi/n_arms*(i-1);
        drot = [	cos(ang)	,	 sin(ang)	,	0
                    -sin(ang)	,	 cos(ang)	,	0
                    0			,	 0			,	1	];
        dpos = zeros(1,3);
        if i == 1
            Color = Colors(2,:);
            if auto
                hold on;
            end
        else
            Color = Colors(1,:);
        end
        draw_square_arm(rot,pos,esc,drot,dpos,Color);
    end
    
    %--- Main Body patch (length units: meters)
    draw_main_body(rot,pos,esc,Colors);

    if auto
        grid on;
        axis equal;
        hold off;
        xlabel('X');
        set(gca,'Ydir', 'reverse')
        ylabel('Y');
        set(gca,'Zdir', 'reverse')
        zlabel('Z');
        view(220,15);
    end

end

function draw_main_body(rot,pos,esc,Colors)
    
    r_sq = 0.07;
    h_sq = 0.01;
    raw_vert_sq = [ r_sq*cos(-pi/4  ) r_sq*sin(-pi/4  ) -h_sq
                    r_sq*cos( pi/4  ) r_sq*sin( pi/4  ) -h_sq
                    r_sq*cos( 3*pi/4) r_sq*sin( 3*pi/4) -h_sq
                    r_sq*cos(-3*pi/4) r_sq*sin(-3*pi/4) -h_sq
                    r_sq*cos(-pi/4  ) r_sq*sin(-pi/4  )  h_sq
                    r_sq*cos( pi/4  ) r_sq*sin( pi/4  )  h_sq
                    r_sq*cos( 3*pi/4) r_sq*sin( 3*pi/4)  h_sq
                    r_sq*cos(-3*pi/4) r_sq*sin(-3*pi/4)  h_sq  ];
    vert_sq = (rot*(raw_vert_sq*esc)')' + ...
                kron(ones(size(raw_vert_sq,1),1),pos);

    face_sq = [   1  2  3  4
                  5  6  7  8
                  1  2  6  5
                  2  3  7  6
                  3  4  8  7
                  4  1  5  8   ];
          
    r_tri = r_sq*cos(pi/4);
    h_tri = h_sq+0.003;
    raw_vert_tri = [r_tri*cos( 0     ) r_tri*sin( 0     ) -h_tri
                    r_tri*cos( 2*pi/3) r_tri*sin( 2*pi/3) -h_tri
                    r_tri*cos(-2*pi/3) r_tri*sin(-2*pi/3) -h_tri
                    r_tri*cos( 0     ) r_tri*sin( 0     )  h_tri
                    r_tri*cos( 2*pi/3) r_tri*sin( 2*pi/3)  h_tri
                    r_tri*cos(-2*pi/3) r_tri*sin(-2*pi/3)  h_tri  ];
    vert_tri = (rot*(raw_vert_tri*esc)')' + ...
                kron(ones(size(raw_vert_tri,1),1),pos);

    face_tri = [  1  2  3 ];
    
    LW = 0.1;
    patch('Vertices',vert_sq ,'Faces',face_sq,'FaceColor',Colors(1,:),'LineWidth',LW);
    patch('Vertices',vert_tri,'Faces',face_tri,'FaceColor',Colors(2,:),'LineWidth',LW);
    
end

function draw_square_arm(rot,pos,esc,drot,dpos,Color)

    side = 0.005;
    len_ini = 0.07*cos(pi/4);
    len_end = 0.2;
    raw_vert = [    len_ini     -side       -side
                    len_end     -side       -side
                    len_end      side       -side
                    len_ini      side       -side
                    len_ini     -side        side
                    len_end     -side        side
                    len_end      side        side
                    len_ini      side        side ];
    vert = (rot*( (drot*(raw_vert*esc)')' + ...
            kron(ones(size(raw_vert,1),1),dpos) )')' + ...
            kron(ones(size(raw_vert,1),1),pos);
    face = [  1  2  3  4
              5  6  7  8
              1  2  6  5
              2  3  7  6
              3  4  8  7
              4  1  5  8 ];
          
    LW = 0.1;
    patch('Vertices',vert,'Faces',face,'FaceColor',Color,'LineWidth',LW);
    
    %--- blade modelling
    t1 = 0:2*pi/7:2*pi;
    r1 = 0.5*len_end;
    h1 = side*1.3;
    l1 = 0.95*len_end;
    raw_vert_h1 = [ l1+[0,sin(t1)*r1]' , [0,cos(t1)*r1]' , ones(length(t1)+1,1)*-h1 ];
    vert_h1 = (rot*( (drot*(raw_vert_h1*esc)')' + ...
            kron(ones(size(raw_vert_h1,1),1),dpos) )')' + ...
            kron(ones(size(raw_vert_h1,1),1),pos);
    face_h1 = zeros(length(vert_h1)-2,3);
    for i = 2:length(vert_h1)-1
        face_h1(i-1,:) = [ 1 , i , i+1 ];
    end
    patch('Vertices',vert_h1,'Faces',face_h1,'FaceColor','none','LineWidth',LW);
      
end
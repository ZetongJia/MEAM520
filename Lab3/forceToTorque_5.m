function tau = forceToTorque_5(F, J)
% FORCETOTORQUE_5 Calculate the joint torques required to exert a 
%   specific set of end-effector forces/torques in a given configuration.
%   Note that you can write this function entirely generally, with no
%   reference to the Lynx.
%
% INPUTS:
%   F - 1 x 6 vector of desired forces/torques (where F(1:3) is the
%       forces and F(4:6) is the torques)
%   J - the 6xN jacobian of the robot in its current configuration  
%
% OUTPUTS:
%   tau - Nx1 vector of joint torques required to generate F.

%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tau = J'*F'

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Your Code Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
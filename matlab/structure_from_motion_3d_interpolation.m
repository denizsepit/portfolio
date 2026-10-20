%% 3D STRUCTURE FROM MOTION (SfM) & NUMERICAL IMAGE INTERPOLATION
% Author: M. Deniz Sepit — Electrical & Electronics Engineering (deniz.sepit@std.ieu.tr)
clear; clc; close all;

% 1. 3D Structure from Motion via Gram-Schmidt & Eigenvalue Decomposition
rng(7); num_pts = 60; num_frames = 12;
theta_gt = linspace(0, 2*pi, num_pts)';
X_true = [cos(theta_gt), sin(theta_gt), 0.5*sin(2*theta_gt)];
X_true = X_true - mean(X_true, 1);

W_meas = zeros(2*num_frames, num_pts);
for f = 1:num_frames
    az = (f-1)*(pi/10); el = 0.35*sin(f*0.4);
    R_cam = [cos(az), -sin(az), 0; sin(az)*cos(el), cos(az)*cos(el), -sin(el)];
    proj = R_cam * X_true' + 0.008*randn(2, num_pts);
    W_meas(2*f-1:2*f, :) = proj - mean(proj, 2);
end

[V_eig, D_eig] = eig(W_meas * W_meas');
[eig_vals, idx] = sort(diag(D_eig), 'descend');
U_3 = V_eig(:, idx(1:3)); S_3 = diag(sqrt(eig_vals(1:3)));
V_3 = (W_meas' * U_3) / S_3;
M_hat = U_3 * sqrt(S_3); S_hat = sqrt(S_3) * V_3';

% Gram-Schmidt Orthogonalization
Q_gs = zeros(3,3); A_ref = M_hat(1:3, 1:3);
for j = 1:3
    v_col = A_ref(:, j);
    for i = 1:(j-1), v_col = v_col - (Q_gs(:,i)'*A_ref(:,j))*Q_gs(:,i); end
    Q_gs(:, j) = v_col / norm(v_col);
end
X_recon = (Q_gs' * S_hat)';

% 2. Root-Finding (Newton-Raphson, Bisection, Secant) for Lens Calibration
k1 = 0.18; k2 = 0.05; r_d = 0.82;
f_lens = @(r) r + k1*r.^3 + k2*r.^5 - r_d;
df_lens = @(r) 1 + 3*k1*r.^2 + 5*k2*r.^4;
r_nr = 0.80;
while abs(f_lens(r_nr)) > 1e-10
    r_nr = r_nr - f_lens(r_nr)/df_lens(r_nr);
end

% 3. Bicubic & Lanczos-3 Spatial Interpolation Kernels
bicubic_k  = @(x) ((1.5*abs(x).^3 - 2.5*abs(x).^2 + 1).*(abs(x)<=1)) + ...
                  ((-0.5*abs(x).^3 + 2.5*abs(x).^2 - 4*abs(x) + 2).*(abs(x)>1 & abs(x)<2));
lanczos3_k = @(x) (x==0) + (x~=0 & abs(x)<3).*(3*sin(pi*x).*sin(pi*x/3))./(pi^2*x.^2+eps);

figure('Color','w');
scatter3(X_recon(:,1), X_recon(:,2), X_recon(:,3), 45, 'r', 'filled');
grid on; axis equal; title('Reconstructed 3D Point Cloud (Gram-Schmidt + Eigenvalues)');

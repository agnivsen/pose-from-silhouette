%% OrthographicProjection. Orthographic projection of a point on a plane
% The plane needs to have a plane.point in [3 x 1] (a point on the plane)
% and a plane.normal in [3 x 1], the surface unit normal of the plane.
% Input points3D must be [3 x N]
%
% Returns:
%  - orthProjWorld: the orthographically projected points in world coordinates (3D)
%  - orthProjCam: the orthographically projected points in camera coordinates (2D)
%
function [orthProjWorld, orthProjCam] = OrthographicProjection(points3D, plane)

    V = plane.point;
    N = plane.normal;

    orthProjWorld = points3D - (((points3D - V).' * N) * N.').';

    camOrientation = [0; 0; 1];
    R_world2cam =vrrotvec2mat(vrrotvec(N, camOrientation));
    points3D_cam = R_world2cam*points3D;

    orthProjCam = points3D_cam(1:2,:);
end
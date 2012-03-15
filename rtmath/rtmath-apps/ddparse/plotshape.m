% Routine to plot a shape.dat file in matlab.
% This routine loads a shape.dat file specified on the command line and
% writes plots to a specified output directory. Several plots are made.
% The first three plots are composite thicknesses when viewed from the x,
% y and z axes. 
% Output is saved in png and eps formats. Automatic matrix dimensioning
% is used to load the files. Slices along the axes may also be specified,
% but more advanced 3d-plotting will be reserved for a future rtmath ui 
% program. Also, this routine is designed to be called from the console.
function plotshape(shapepath, outputdir)
	disp('Rendering plots');
	data = importdata(shapepath, ' ', 7);
	% x is column 2, y is 3, z is 4
	xmax = max(data.data(:,2));
	xmin = min(data.data(:,2));
	ymax = max(data.data(:,3));
	ymin = min(data.data(:,3));
	zmax = max(data.data(:,4));
	zmin = min(data.data(:,4));

	% Cannot just plot directly
	% Must translate to a zeroed array

	% Set dimensions
	sz = size(data.data);
	xa = zeros(ymax-ymin+1,zmax-zmin+1);
	ya = zeros(xmax-xmin+1,zmax-zmin+1);
	za = zeros(xmax-xmin+1,ymax-ymin+1);

	% Set composite thickness matrices
	for i=1:sz(1)
		x = data.data(i,2);
		y = data.data(i,3);
		z = data.data(i,4);

		% Scaled coordinates
		xs = x - xmin + 1;
		ys = y - ymin + 1;
		zs = z - zmin + 1;

		xa(ys,zs) = xa(ys,zs) + 1;
		ya(xs,zs) = ya(xs,zs) + 1;
		za(xs,ys) = za(xs,ys) + 1;
	end

	% Plot composite matrices
	h = pcolor(xa);
	title(strcat(data.textdata(1,1), '- x composite'));
	colorbar;
	set(h, 'LineStyle', 'none');
	xlabel('y');
	ylabel('z');
	print(gcf, '-dpng', strcat(outputdir,'/shape-xc.png'));

	h = pcolor(ya);
	title(strcat(data.textdata(1,1), '- y composite'));
	colorbar;
	set(h, 'LineStyle', 'none');
	xlabel('x');
	ylabel('z');
	print(gcf, '-dpng', strcat(outputdir,'/shape-yc.png'));

	h = pcolor(za);
	title(strcat(data.textdata(1,1), '- z composite'));
	colorbar;
	set(h, 'LineStyle', 'none');
	xlabel('x');
	ylabel('y');
	print(gcf, '-dpng', strcat(outputdir,'/shape-zc.png'));

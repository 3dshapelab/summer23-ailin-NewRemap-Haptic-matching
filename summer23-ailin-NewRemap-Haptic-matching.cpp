#include "spring23-ailin-motionRemap-matching.h"

/***** SOUNDS *****/
void beepOk(int tone)
{

	switch (tone)
	{

	case 1: //high pitch beep
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-8_lowpass.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 4: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-440-pluck.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 2: //reject like buzz
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-10.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 3: //reject short
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-reject.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 5: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-highBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 6: //mellow and good for trials
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-lowBubblePop.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 8: //spoken mirror
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-mirror.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 15:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-rising.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 16:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-falling.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 17:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-440-pluck-5below.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 18: // light click
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\beep-click3MS.wav",
			NULL, SND_FILENAME | SND_ASYNC);
		break;

	case 19:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-one.wav", NULL, SND_FILENAME | SND_ASYNC);
		break;
	case 20:
		PlaySound((LPCSTR)"C:\\cncsvision\\data\\beep\\spoken-two.wav", NULL, SND_FILENAME | SND_ASYNC);
	}
	return;
}


double normalCDF(double value)
{
	double M_SQRT1_2 = sqrt(0.5);
	return 0.5 * erfc(-value * M_SQRT1_2);
}


double blurEdge(double dropoff, double pointDist, double intersectDist, double baseCol, double maxCol) {
	double addCol = maxCol - baseCol;
	return(((pointDist / intersectDist) < dropoff) ? baseCol : (baseCol + addCol * normalCDF(((pointDist / intersectDist - dropoff) / (1 - dropoff) * 6) - 3)));
}

double getZ(double shapeHeight, double shapeDepth, double Y) {

	double Z;

	Z = shapeDepth * cos(M_PI * Y / shapeHeight);

	return (Z);
}

double getTg(double shapeHeight, double shapeDepth, double Y) {
	return (-shapeDepth * sin(M_PI * Y / shapeHeight) * M_PI / shapeHeight);
}



double NewtonSolver_fz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = z / Depth - cos(zCoeff * (z - distShapeToEye));
	return val;
}

double NewtonSolver_dfz(double z, double Depth, double zCoeff, double distShapeToEye) {
	double val = 1 / Depth + sin(zCoeff * (z - distShapeToEye)) * zCoeff;
	return val;
}

double SolveForZ_projected(double theHeight, double newDepth, double distShapeToEye, double y0, double z0) {

	double z_new, z, f_z, df_z;
	double C = M_PI * y0 / (theHeight * (z0 - distShapeToEye));

	z = z0;

	for (int i = 0; i < 100; i++) {

		f_z = NewtonSolver_fz(z, newDepth, C, distShapeToEye);
		df_z = NewtonSolver_dfz(z, newDepth, C, distShapeToEye);

		if (abs(f_z) < 1e-10) {

			break;
		}
		else if (abs(df_z) < 1e-10) {

			break;
		}
		else {
			z_new = z - f_z / df_z;
			z = z_new;
		}
	}

	if (abs(z - z0) > 40)
		z = 0;

	return z;

}




void scanCurve(double shapeHeight, double shapeDepth, CurveYLMap& output_curve_ylmap) {
	output_curve_ylmap = {};

	double y, z, l, y_prev, z_prev;
	double step_size = (shapeHeight / (nr_curve_map - 1));

	output_curve_ylmap.curve_depth = shapeDepth;
	output_curve_ylmap.curve_height = shapeHeight;
	output_curve_ylmap.step_size = step_size;

	// the first point
	y = -shapeHeight / 2;
	z = 0;
	l = 0;
	y_prev = y, z_prev = z;
	output_curve_ylmap.y_vec.push_back(y);
	output_curve_ylmap.l_vec.push_back(l);


	for (int j = 1; j < nr_curve_map; j++) {
		y = -shapeHeight / 2 + j * step_size;
		z = getZ(shapeHeight, shapeDepth, y);
		l = l + sqrt(pow(y - y_prev, 2) + pow(z - z_prev, 2));

		output_curve_ylmap.y_vec.push_back(y);
		output_curve_ylmap.l_vec.push_back(l);

		y_prev = y; z_prev = z;
	}


}


void generateTexDots_hybrid(float TM_X, float TM_Y, float dotDensity, float dotRadius, float dotSeparationRatio, int nr_X_Lattice, float dotJitterScale_Lattice, TextureDotsData& outputTexDots) {

	outputTexDots = {};

	outputTexDots.TexMapSize = Vec2{ TM_X, TM_Y };
	outputTexDots.Radius = dotRadius;
	outputTexDots.margin_y = R_blur_fac * dotRadius;

	std::uniform_real_distribution<float> dist(0.f, 1.f);

	// step 1: generate dots
	vector<Vec2> dc_vec;

	int num_dot = TM_X * TM_Y * dotDensity;

	float dot_separation = dotSeparationRatio * 2 * dotRadius;
	float dotSeparationRatio_adjust = dotSeparationRatio;

	int genTexAttemps = 0;
	int num_runs = 0;
	int change_count = 0;

	int nr_X = nr_X_Lattice;
	int nr_Y = floor(TM_Y * nr_X / TM_X);

	if (nr_Y % 2 == 0) {
		nr_Y++;
	}

	float lat_step_x = TM_X / (float)nr_X;
	float lat_step_y = TM_Y / (float)nr_Y;

	for (int j = 0; j < nr_Y; j++) {
		for (int i = 0; i < nr_X; i++) {
			float rx = dist(rng);
			float ry = dist(rng);
			float cx_lat = ((rx - 0.5) * dotJitterScale_Lattice + i + 0.5) * lat_step_x;
			float cy_lat = ((ry - 0.5) * dotJitterScale_Lattice + j + 0.5) * lat_step_y;

			dc_vec.push_back(Vec2{ cx_lat, cy_lat });
		}
	}

	for (int i_dot = nr_X * nr_Y; i_dot < num_dot; i_dot++) {

		num_runs++;

		if (num_runs > 10000) {

			genTexAttemps++;

			if (genTexAttemps > 10000) {

				cerr << "can't generate texture" << endl;
				exit(0);
			}
			else {
				num_runs = 0;
				dc_vec.clear();

				for (int j = 0; j < nr_Y; j++) {
					for (int i = 0; i < nr_X; i++) {

						float rx = dist(rng);
						float ry = dist(rng);
						float cx_lat = ((rx - 0.5) * dotJitterScale_Lattice + i + 0.5) * lat_step_x;
						float cy_lat = ((ry - 0.5) * dotJitterScale_Lattice + j + 0.5) * lat_step_y;

						dc_vec.push_back(Vec2{ cx_lat, cy_lat });
					}
				}

				i_dot = nr_X * nr_Y;

				change_count = floor(genTexAttemps / (float)50);
				dotSeparationRatio_adjust = dotSeparationRatio_adjust - 0.00001 * (float)change_count * (float)num_dot;
				//dotSeparationRatio_adjust = dotSeparationRatio_adjust - 0.000005 * (float)num_dot;
				dot_separation = dotSeparationRatio_adjust * 2 * dotRadius;

			}
		}

		// pick random xy values for the circle center
		float cx = dist(rng); // 0-1
		float cy = dist(rng); // 0-1
		cx *= TM_X;
		cy *= TM_Y;

		// checking whether the current circle intersects withe the previous circles already pushed
		bool intersect = false;
		for (int k = 0; k < dc_vec.size(); k++) {
			Vec2 prev_c = dc_vec[k];
			if ((cx - prev_c.x) * (cx - prev_c.x) + (cy - prev_c.y) * (cy - prev_c.y) < dot_separation * dot_separation) {
				intersect = true;
				break;
			}
		}

		// if intersect, then break and pick a new circle center
		if (intersect) {
			i_dot--;
			continue;
		}

		// if not intersect, add this circle to the circles vector
		dc_vec.push_back(Vec2{ cx, cy });
	}


	// sort the dot by their y
	int n_dots = dc_vec.size();

	vector<float> dc_y_vec;
	for (int k = 0; k < n_dots; k++) {
		dc_y_vec.push_back(dc_vec[k].y);
	}
	vector<int> sortedDC_ind_vec(n_dots);
	std::iota(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), 0); //Initializing
	std::sort(sortedDC_ind_vec.begin(), sortedDC_ind_vec.end(), [&](int i, int j) {return dc_y_vec[i] < dc_y_vec[j]; });

	// fill in TexDots dot center vectors in order of low y to high y
	float x_offset = TM_X / 2.;
	float y_offset = TM_Y / 2.;
	for (int k = 0; k < n_dots; k++) {
		Vec2 dc_temp = dc_vec[sortedDC_ind_vec[k]];
		outputTexDots.dot_center_vec.push_back(Vec2{ dc_temp.x - x_offset, dc_temp.y - y_offset });
	}

}



double mapLtoY(const CurveYLMap& inputYLMap, double TargetL, int i_int) {

	int i_c = i_int;
	double depth = inputYLMap.curve_depth;
	double height = inputYLMap.curve_height;
	double step_y_ylmap = inputYLMap.step_size;
	double y, l_diff, dldy;

	for (int i = 0; i < 100; i++) {

		l_diff = TargetL - inputYLMap.l_vec[i_c];

		y = inputYLMap.y_vec[i_c];

		double k = getTg(height, depth, y);
		dldy = sqrt(1 + k * k);

		if (abs(l_diff) <= step_y_ylmap * dldy) {

			break;
		}
		else {
			y = y + l_diff / dldy;

			i_c = (y + height / 2) / step_y_ylmap;


			if (i_c > (nr_curve_map - 1))
				i_c = nr_curve_map - 1;


			if (i_c < 0)
				i_c = 0;

		}
	}

	y = y + l_diff / dldy;

	return y;
}


double mapYtoL(const CurveYLMap& inputYLMap, double input_y) {
	double mapStpSz = inputYLMap.step_size;
	double i_c_val = (input_y - inputYLMap.y_vec[0]) / mapStpSz;
	int i_c = round(i_c_val);
	int i_c0 = floor(i_c_val);
	int i_c1 = ceil(i_c_val);
	double l = inputYLMap.l_vec[i_c0] + (input_y - inputYLMap.y_vec[i_c0]) * (inputYLMap.l_vec[i_c1] - inputYLMap.l_vec[i_c0]) / mapStpSz;
	return l;
}



Vector3d projectPoint(double shapeHeight, double newDepth, double distShapeToEye, Vector3d fromPoint) {

	Vector3d ToPoint = fromPoint;
	if (abs(abs(fromPoint.y()) - shapeHeight / 2) > 0.01) {
		double z = SolveForZ_projected(shapeHeight, newDepth, distShapeToEye, fromPoint.y(), fromPoint.z());
		double w = (z - distShapeToEye) / (fromPoint.z() - distShapeToEye);
		ToPoint = Vector3d(w * fromPoint.x(), w * fromPoint.y(), z);
	}

	return ToPoint;
}




void buildSurface_congruent(double shapeWidth, double shapeHeight, double shapeDepth, double distShapeToEye, VerticesData& vertices_data, double contourPanelSeparation, ContourData& contour_data) {

	// part 1: generate TexDots
	CurveYLMap ylMap_Text;
	scanCurve(shapeHeight, shapeDepth, ylMap_Text);
	float l_text = ylMap_Text.l_vec.back();
	TextureDotsData Tex_Dots_text;
	generateTexDots_hybrid(shapeWidth, 1.25 * l_text, Tex_dot_density, Tex_dot_radius, Tex_dot_separation_ratio, nr_X_Lat_TexDot, jitter_Lat_TexDot, Tex_Dots_text);

	// build vertices
	CurvePtsData y_curve_data_text;
	int nr_points_height = buildCurve_byDelL(ylMap_Text, y_curve_data_text);
	buildSurfaceVertices_congruent(shapeWidth,  y_curve_data_text, Tex_Dots_text, vertices_data);
	buildContour(contourPanelSeparation, y_curve_data_text, y_curve_data_text, distShapeToEye, contour_data);

}

void buildSurface_incongruent(double shapeWidth, double shapeHeight, double dispDepth, double textDepth, double distShapeToEye, VerticesData& vertices_data, double contourPanelSeparation, ContourData& contour_data) {


	// part 1: generate TexDots
	CurveYLMap ylMap_Text;
	scanCurve(shapeHeight, textDepth, ylMap_Text);	
	float l_text = ylMap_Text.l_vec.back();
	TextureDotsData Tex_Dots_text, Tex_Dots_disp;
	generateTexDots_hybrid(shapeWidth, 1.25 * l_text, Tex_dot_density, Tex_dot_radius, Tex_dot_separation_ratio, nr_X_Lat_TexDot, jitter_Lat_TexDot, Tex_Dots_text);

	// part 1: project TexDots to Disp Surface
	CurveYLMap ylMap_Disp;
	scanCurve(shapeHeight, dispDepth, ylMap_Disp);


	CurvePtsData y_curve_data_text, y_curve_data_disp;
	int nr_points_height = buildCurve_byDelL(ylMap_Text, y_curve_data_text);
	projectCurve(ylMap_Disp, distShapeToEye, y_curve_data_text, y_curve_data_disp);
	buildSurfaceVertices_TexDotsOnText(shapeWidth, y_curve_data_disp, y_curve_data_text, distShapeToEye, Tex_Dots_text, vertices_data);
	buildContour(contourPanelSeparation, y_curve_data_disp, y_curve_data_text, distShapeToEye, contour_data);

}



void buildSurfaceVertices_congruent(double shapeWidth, const CurvePtsData& textYCurve, TextureDotsData& TexDotsOnText, VerticesData& vertices_data) {

	vertices_data = {};

	GLuint i_ind = 0;
	int nr_J = textYCurve.y_vec.size();
	double stpsz_I = shapeWidth / (nr_points_width - 1);
	double height = textYCurve.curve_height;
	double depth_text = textYCurve.curve_depth;
	float x_offset = TexDotsOnText.TexMapSize.x / 2;

	// for texture colors
	float vertex_col = 1.0f;
	int nr_dots = TexDotsOnText.dot_center_vec.size();
	float R = TexDotsOnText.Radius;
	float L_start = -textYCurve.l_vec.back() / 2.;
	float l_margin = TexDotsOnText.margin_y;

	int TexDot_Ind_L = 0;
	int TexDot_Ind_H = 0;
	vector<Vec2> nearTexDots_dc_vec;

	for (int jj = 0; jj < nr_J; jj++) {


		float y_t = textYCurve.y_vec[jj];
		float z_t = textYCurve.z_vec[jj];

		float tg_t = getTg(height, depth_text, y_t);

		double L_t = textYCurve.l_vec[jj] + L_start;

		// find the dots that are near l_t
		while ((TexDotsOnText.dot_center_vec[TexDot_Ind_L].y < ((float)L_t - l_margin)) && TexDot_Ind_L < (nr_dots - 1)) {
			TexDot_Ind_L++;
		}


		TexDot_Ind_H = TexDot_Ind_L;
		while ((TexDotsOnText.dot_center_vec[TexDot_Ind_H].y < ((float)L_t + l_margin)) && TexDot_Ind_H < (nr_dots - 1)) {
			TexDot_Ind_H++;
		}

		nearTexDots_dc_vec.clear();
		for (int k = TexDot_Ind_L; k < TexDot_Ind_H + 1; k++) {
			nearTexDots_dc_vec.push_back(TexDotsOnText.dot_center_vec[k]);
		}
		int nr_dots_near = nearTexDots_dc_vec.size();


		for (int ii = 0; ii < nr_points_width; ii++) {

			double pt_x = stpsz_I * ii - x_offset;
			double pt_y = L_t;
			vertex_col = 1.0f;



			for (int k = 0; k < nr_dots_near; k++) {

				double pt_val = sqrt((pt_x - nearTexDots_dc_vec[k].x) * (pt_x - nearTexDots_dc_vec[k].x) +
					(pt_y - nearTexDots_dc_vec[k].y) * (pt_y - nearTexDots_dc_vec[k].y));

				if (pt_val < R_blur_fac * R) {
					double vertex_col_tentative = blurEdge(drop_off_rate, pt_val, R_blur_fac * R, 0.0, 1.0);
					vertex_col = float(vertex_col_tentative);
					break;
				}
			}

			float x_t = (pt_x);


			vertices_data.vertices_vec.push_back(x_t);
			vertices_data.vertices_vec.push_back(y_t);
			vertices_data.vertices_vec.push_back(z_t);

			vertices_data.light_normals_vec.push_back(0);
			vertices_data.light_normals_vec.push_back(-tg_t);
			vertices_data.light_normals_vec.push_back(1);


			vertices_data.colors_vec.push_back(vertex_col_min + vertex_col * (vertex_col_max - vertex_col_min));
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			if (ii < nr_points_width - 1 && jj < nr_J - 1) {

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);

			}

			i_ind++;
		}
	}

}

void buildSurfaceVertices_TexDotsOnText(double shapeWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, double distShapeToEye, TextureDotsData& TexDotsOnText, VerticesData& vertices_data) {
	vertices_data = {};

	GLuint i_ind = 0;
	int nr_J = dispYCurve.y_vec.size();
	double stpsz_I = shapeWidth / (nr_points_width - 1);
	double height = textYCurve.curve_height;
	double depth_text = textYCurve.curve_depth;
	float x_offset = TexDotsOnText.TexMapSize.x / 2;

	// for texture colors
	float vertex_col = 1.0f;
	int nr_dots = TexDotsOnText.dot_center_vec.size();
	float R = TexDotsOnText.Radius;
	float L_start = -textYCurve.l_vec.back() / 2.;
	float l_margin = TexDotsOnText.margin_y;

	int TexDot_Ind_L = 0;
	int TexDot_Ind_H = 0;
	vector<Vec2> nearTexDots_dc_vec;

	for (int jj = 0; jj < nr_J; jj++) {

		float y_d = dispYCurve.y_vec[jj];
		float z_d = dispYCurve.z_vec[jj];

		float y_t = textYCurve.y_vec[jj];
		float z_t = textYCurve.z_vec[jj];

		float tg_t = getTg(height, depth_text, y_t);

		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		float x_d;

		double L_t = textYCurve.l_vec[jj] + L_start;

		// find the dots that are near L_t
		while (((TexDotsOnText.dot_center_vec[TexDot_Ind_L].y) < ((float)L_t - l_margin)) && TexDot_Ind_L < (nr_dots - 1)) {
			TexDot_Ind_L++;
		}


		TexDot_Ind_H = TexDot_Ind_L;
		while (((TexDotsOnText.dot_center_vec[TexDot_Ind_H].y) < ((float)L_t + l_margin)) && TexDot_Ind_H < (nr_dots - 1)) {
			TexDot_Ind_H++;
		}

		nearTexDots_dc_vec.clear();
		for (int k = TexDot_Ind_L; k < TexDot_Ind_H + 1; k++) {
			nearTexDots_dc_vec.push_back(TexDotsOnText.dot_center_vec[k]);
		}
		int nr_dots_near = nearTexDots_dc_vec.size();


		for (int ii = 0; ii < nr_points_width; ii++) {

			double pt_x = stpsz_I * ii - x_offset;
			double pt_y = L_t;
			vertex_col = 1.0f;



			for (int k = 0; k < nr_dots_near; k++) {

				double pt_val = sqrt((pt_x - nearTexDots_dc_vec[k].x) * (pt_x - nearTexDots_dc_vec[k].x) +
					(pt_y - nearTexDots_dc_vec[k].y) * (pt_y - nearTexDots_dc_vec[k].y));

				if (pt_val < R_blur_fac * R) {
					double vertex_col_tentative = blurEdge(drop_off_rate, pt_val, R_blur_fac * R, 0.0, 1.0);
					vertex_col = float(vertex_col_tentative);
					break;
				}
			}

			x_d = w * pt_x;


			vertices_data.vertices_vec.push_back(x_d);
			vertices_data.vertices_vec.push_back(y_d);
			vertices_data.vertices_vec.push_back(z_d);

			vertices_data.light_normals_vec.push_back(0);
			vertices_data.light_normals_vec.push_back(-tg_t);
			vertices_data.light_normals_vec.push_back(1);


			vertices_data.colors_vec.push_back(vertex_col_min + vertex_col * (vertex_col_max - vertex_col_min));
			vertices_data.colors_vec.push_back(0);
			vertices_data.colors_vec.push_back(0);

			if (ii < nr_points_width - 1 && jj < nr_J - 1) {

				// using vector
				vertices_data.indices_draw_triangle_vec.push_back(i_ind);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);

				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + 1);
				vertices_data.indices_draw_triangle_vec.push_back(i_ind + nr_points_width + 1);

			}

			i_ind++;
		}
	}
}



void buildContour(double ContourWidth, const CurvePtsData& dispYCurve, const CurvePtsData& textYCurve, float distShapeToEye, ContourData& new_contours_vert) {

	new_contours_vert = {};

	for (int i_v = 0; i_v < dispYCurve.y_vec.size(); i_v++) {

		float x_t_L = -ContourWidth / 2;
		float x_t_R = ContourWidth / 2;

		double z_t = textYCurve.z_vec[i_v];

		float y_d = dispYCurve.y_vec[i_v];
		float z_d = dispYCurve.z_vec[i_v];

		//float w = (distShapeToEye - z_d) / (distShapeToEye - (z_t + z_d) / 2.0);
		float w = (distShapeToEye - z_d) / (distShapeToEye - z_t);
		float x_v_L = w * x_t_L;
		float x_v_R = w * x_t_R;

		new_contours_vert.vert_Lcontour.push_back(Vector3f(x_v_L, y_d, z_d));
		new_contours_vert.vert_Rcontour.push_back(Vector3f(x_v_R, y_d, z_d));
	}
}


int buildCurve_byDelL(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve) {

	output_curve = {};

	double depth = input_curve_ylmap.curve_depth;
	output_curve.curve_depth = depth;
	double height = input_curve_ylmap.curve_height;
	output_curve.curve_height = height;
	double ylmap_stpsz = input_curve_ylmap.step_size;
	double y_min = input_curve_ylmap.y_vec[0];

	// get an even number of steps
	double l_total = input_curve_ylmap.l_vec.back();
	int nr_steps = (int)(l_total / del_l);
	if (nr_steps % 2 == 1)
		nr_steps++;


	double del_l_adjust = l_total / nr_steps;

	int ind_midway = nr_steps / 2;
	int i_temp = 0;

	double y, l;

	for (int k = 0; k < ind_midway; k++) {

		l = k * del_l_adjust;
		y = mapLtoY(input_curve_ylmap, l, i_temp);
		i_temp = (y - y_min) / ylmap_stpsz;
		output_curve.y_vec.push_back(y);
		output_curve.z_vec.push_back(getZ(height, depth, y));
		output_curve.l_vec.push_back(l);

	}

	l = l_total / 2.;
	y = 0;

	output_curve.y_vec.push_back(y);
	output_curve.z_vec.push_back(depth);
	output_curve.l_vec.push_back(l);

	for (int k = 1; k < ind_midway + 1; k++) {
		int i_c = ind_midway - k;
		l = l + del_l_adjust;

		output_curve.y_vec.push_back(-output_curve.y_vec[i_c]);
		output_curve.z_vec.push_back(output_curve.z_vec[i_c]);
		output_curve.l_vec.push_back(l);

	}

	output_curve.step_size = del_l_adjust;


	return output_curve.y_vec.size();
}

int buildCurve_byDelY(const CurveYLMap& input_curve_ylmap, CurvePtsData& output_curve) {

	output_curve = {};

	double depth = input_curve_ylmap.curve_depth;
	output_curve.curve_depth = depth;
	double height = input_curve_ylmap.curve_height;
	output_curve.curve_height = height;
	double y, l;

	double stpsz_J = height / (nr_points_height_default - 1);
	double stpsz_ycurve_precise = height / (nr_curve_map - 1);
	for (int j = 0; j < nr_points_height_default; j++) {
		int k = stpsz_J * j / stpsz_ycurve_precise;
		y = input_curve_ylmap.y_vec[k];
		output_curve.y_vec.push_back(y);
		output_curve.z_vec.push_back(getZ(height, depth, y));
		output_curve.l_vec.push_back(input_curve_ylmap.l_vec[k]);
	}

	return output_curve.y_vec.size();
}

void projectCurve(const CurveYLMap& curve_map_proj, double distShapeToEye, const CurvePtsData& origin_curve, CurvePtsData& output_curve_proj) {

	output_curve_proj = {};

	double newDepth = curve_map_proj.curve_depth;
	double height = curve_map_proj.curve_height;
	double step_y_ylmap = curve_map_proj.step_size;

	output_curve_proj.curve_height = height;
	output_curve_proj.curve_depth = newDepth;

	double y_p, z_p, l_p, tg_p;


	for (int jj = 0; jj < origin_curve.y_vec.size(); jj++) {

		double y_o = origin_curve.y_vec[jj];
		double z_o = origin_curve.z_vec[jj];
		z_p = SolveForZ_projected(height, newDepth, distShapeToEye, y_o, z_o);
		double w = (z_p - distShapeToEye) / (z_o - distShapeToEye);
		y_p = w * y_o;
		int i_c = (y_p - curve_map_proj.y_vec[0]) / step_y_ylmap;
		l_p = curve_map_proj.l_vec[i_c];

		output_curve_proj.y_vec.push_back(y_p);
		output_curve_proj.z_vec.push_back(z_p);
		output_curve_proj.l_vec.push_back(l_p);

	}

}




float adjustAmbient(double textDepth, float maxInt, double rateAmbvsDiff_flat, double rateAmbvsDiff_deep, double Depth_flat, double Depth_deep) {

	double rateAmbvsDiff_new = rateAmbvsDiff_flat + (rateAmbvsDiff_deep - rateAmbvsDiff_flat) * (textDepth - Depth_flat) / (Depth_deep - Depth_flat);
	float newAmbient = maxInt * (rateAmbvsDiff_new / (rateAmbvsDiff_new + 1));

	return newAmbient;
}

void drawVertices(bool isStandard, double distShapeToEye) {

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	//setting the light
	glShadeModel(GL_SMOOTH); // enable Smooth Shading
	glEnable(GL_LIGHTING); // enable lighting
	glEnable(GL_LIGHT1);
	glEnable(GL_NORMALIZE); //so we don't need to normalize our normal for surfaces	

	// activate and specify pointer to vertex array
	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_NORMAL_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, -distShapeToEye);

	if (isStandard) {

		glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient_std); //setup the ambient light
		glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse_std); //setup the diffuse light
		glLightfv(GL_LIGHT1, GL_POSITION, LightPosition); //position the light

		glVertexPointer(3, GL_FLOAT, 0, &my_vertices_data_std.vertices_vec[0]);
		glNormalPointer(GL_FLOAT, 0, &my_vertices_data_std.light_normals_vec[0]); //
		glColorPointer(3, GL_FLOAT, 0, &my_vertices_data_std.colors_vec[0]);

		glDrawElements(GL_TRIANGLES, my_vertices_data_std.indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &my_vertices_data_std.indices_draw_triangle_vec[0]);

		// deactivate vertex arrays after drawing
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);

		glDisable(GL_LIGHTING);
		drawContours(my_contour_data_std);
	}
	else {
		//setting the light
		glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient_cmp); //setup the ambient light
		glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse_cmp); //setup the diffuse light
		glLightfv(GL_LIGHT1, GL_POSITION, LightPosition); //position the light

		glVertexPointer(3, GL_FLOAT, 0, &my_vertices_data_cmp.vertices_vec[0]);
		glNormalPointer(GL_FLOAT, 0, &my_vertices_data_cmp.light_normals_vec[0]); //
		glColorPointer(3, GL_FLOAT, 0, &my_vertices_data_cmp.colors_vec[0]);

		glDrawElements(GL_TRIANGLES, my_vertices_data_cmp.indices_draw_triangle_vec.size(), GL_UNSIGNED_INT, &my_vertices_data_cmp.indices_draw_triangle_vec[0]);

		// deactivate vertex arrays after drawing
		glDisableClientState(GL_VERTEX_ARRAY);
		glDisableClientState(GL_NORMAL_ARRAY);
		glDisableClientState(GL_COLOR_ARRAY);

		glDisable(GL_LIGHTING);
		drawContours(my_contour_data_cmp);
	}

	glPopMatrix();
}

void drawContours(const ContourData& contours_vert) {

	int n;
	float panel_width = 40;
	float panel_height_extra = 20;

	glTranslated(0, 0, 1);
	n = int(contours_vert.vert_Lcontour.size());
	glColor3f(0.0f, 0.0f, 0.0f);
	if (n > 0) {

		// Right panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Rcontour.at(0)[0] + panel_width, contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(0)[0], contours_vert.vert_Rcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Rcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0] + panel_width, contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Rcontour.at(i)[0], contours_vert.vert_Rcontour.at(i)[1], contours_vert.vert_Rcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0] + panel_width, contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Rcontour.at(n - 1)[0], contours_vert.vert_Rcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Rcontour.at(n - 1)[2]); //1

		glEnd();

		// Left panels
		glBegin(GL_QUAD_STRIP);

		glVertex3f(contours_vert.vert_Lcontour.at(0)[0], contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(0)[0] - panel_width, contours_vert.vert_Lcontour.at(0)[1] - panel_height_extra, contours_vert.vert_Lcontour.at(0)[2]); //1

		for (int i = 0; i < n; i++)
		{
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0], contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //0
			glVertex3f(contours_vert.vert_Lcontour.at(i)[0] - panel_width, contours_vert.vert_Lcontour.at(i)[1], contours_vert.vert_Lcontour.at(i)[2]); //1

		}

		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0], contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //0
		glVertex3f(contours_vert.vert_Lcontour.at(n - 1)[0] - panel_width, contours_vert.vert_Lcontour.at(n - 1)[1] + panel_height_extra, contours_vert.vert_Lcontour.at(n - 1)[2]); //1

		glEnd();
	}

}

void drawFixation(double displayDist) {
	// draws a small fixation cross at the center of the display
	glEnable(GL_LINE_SMOOTH);
	glColor3f(1.0f, 0.0f, 0.0f);
	glLineWidth(3.f);

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, displayDist);
	double cross_length = 5;
	glBegin(GL_LINES);
	glVertex3d(cross_length / 2, 0, 0);
	glVertex3d(-cross_length / 2, 0, 0);
	glVertex3d(0, -cross_length / 2., 0);
	glVertex3d(0, cross_length / 2., 0);
	glEnd();
	glPopMatrix();
	glDisable(GL_LINE_SMOOTH);
}

// This function seems to be used to shut down the system after use
void shutdown() {
	cout << "shutting down" << endl;
	responseFile.close(); // close this object
	if (resetScreen_betweenRuns) {
		homeEverything(5000, 4500);
	}
	cleanup();
	exit(0);
}
void cleanup()
{
	// Stop the optotrak
	optotrak.stopCollection();
}
void initProjectionScreen(double _focalDist, const Affine3d& _transformation, bool synchronous)
{
	focalDistance = _focalDist;
	screen.setWidthHeight(SCREEN_WIDE_SIZE, SCREEN_WIDE_SIZE * SCREEN_HEIGHT / SCREEN_WIDTH);
	screen.setOffset(alignmentX, alignmentY);
	screen.setFocalDistance(_focalDist);
	screen.transform(_transformation);
	cam.init(screen);
	if (synchronous)
		moveScreenAbsolute(_focalDist, homeFocalDistance, 4500);
	else
		moveScreenAbsoluteAsynchronous(_focalDist, homeFocalDistance, 4500);
}


// run a method to define a vector that holds marker positions 
void initOptotrak()
{
	initRotationM();
	optotrak.setTranslation(calibration);

	if (optotrak.init(LastAlignedFile, OPTO_NUM_MARKERS, OPTO_FRAMERATE, OPTO_MARKER_FREQ, OPTO_DUTY_CYCLE, OPTO_VOLTAGE) != 0)
	{
		cerr << "Something during Optotrak initialization failed, press ENTER to continue. A error log has been generated, look \"opto.err\" in this folder" << endl;
		cin.ignore(1E6, '\n');
		exit(0);
	}

	for (int i = 0; i < 10; i++) {
		updateTheMarkers();
	}

}

void updateTheMarkers()
{
	optotrak.updateMarkers();
	markers = optotrak.getAllMarkers();

	for (int i = 1; i <= OPTO_NUM_MARKERS; i++)
	{
		markers.at(i).p = rotationM * markers.at(i).p;
	}

}
// Initialize motors for moving screen around
void initMotors()
{
	//specify the speed for (objects,screen)
	if (resetScreen_betweenRuns) {
		homeEverything(5000, 4500);
	}

}

// Method that initializes the openGL parameters needed for creating the stimuli. 
// seems like this is not changed for each experiment (maybe for different experimental setup eg monitor)
void initRendering()
{

	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/* Set depth buffer clear value */
	glClearDepth(1.0);
	/* Enable depth test */
	glEnable(GL_DEPTH_TEST);
	/* Set depth function */
	glDepthFunc(GL_LEQUAL);
	// scommenta solo se vuoi attivare lo shading degli stimoli

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glLineWidth(1.5);

}


void initVariables()
{
	interoculardistance = atof(parameters.find("IOD").c_str());
	display_distance = str2num<double>(parameters.find("dispDepth"));

	// eye coordinates

	eyeRight = Vector3d(interoculardistance / 2, 0, 0);
	eyeLeft = Vector3d(-interoculardistance / 2, 0, 0);

	eyeMiddle = Vector3d(0, 0, 0);

	//stimulus_height = tan((DEG2RAD * visual_angle) / 2) * 2 * (abs(display_distance));

	//stimulus_width = ratio_width_height * stimulus_height;
	//stimulus_visiblewidth = ratio_visiblewidth_height * stimulus_height;


}


void initBlock(int blockNumber)
{
	string tempFileName;
	ifstream tempparFile;
	use_par = stairOrder[(blockNumber-1)];
	cout << int(use_par) << endl;
	switch (use_par) {
	case A0:
	{
		tempFileName = experiment_directory + "matching_staircases/parameters_spring23-ailin-motionRemap-matching-stairA0.txt";
		tempparFile.open(tempFileName.c_str());
		par_s1.loadParameterFile(tempparFile);
		trial.init(par_s1);
		ascending = trial.getCurrent().second->getCurrentStaircase()->getAscending();
		trial.next((bool)ascending);
	}
		break;

	case A1:
		tempFileName = experiment_directory + "matching_staircases/parameters_spring23-ailin-motionRemap-matching-stairA1.txt";
		tempparFile.open(tempFileName.c_str());
		par_s2.loadParameterFile(tempparFile);
		trial.init(par_s2);
		ascending = trial.getCurrent().second->getCurrentStaircase()->getAscending();
		trial.next((bool)ascending);
		break;

	case B0:
		tempFileName = experiment_directory + "matching_staircases/parameters_spring23-ailin-motionRemap-matching-stairB0.txt";
		tempparFile.open(tempFileName.c_str());
		par_s3.loadParameterFile(tempparFile);
		trial.init(par_s3);
		ascending = trial.getCurrent().second->getCurrentStaircase()->getAscending();
		trial.next((bool)ascending);
		break;

	case B1:
		tempFileName = experiment_directory + "matching_staircases/parameters_spring23-ailin-motionRemap-matching-stairB1.txt";
		tempparFile.open(tempFileName.c_str());
		par_s4.loadParameterFile(tempparFile);
		trial.init(par_s4);
		ascending = trial.getCurrent().second->getCurrentStaircase()->getAscending();
		trial.next((bool)ascending);
		break;
	}


	//trialNum = 1;
}

// Initialize the streams, open the file and write to it
void initStreams()
{
	// Initialize the parameter file starting from the file parameters.txt, if the file does not exist, it tells you

	parametersFile.open(parametersFileName.c_str());
	parameters.loadParameterFile(parametersFile);

	// Subject name
	subjectName = parameters.find("SubjectName");
	string session = parameters.find("Session");
	sessionNum = str2num<int>(parameters.find("Session"));

	int roll_a_dice = rand() % 4;
	switch (roll_a_dice) {
	case 1:
		stairOrder[0] = A1;
		stairOrder[1] = B0;
		stairOrder[2] = A0;
		stairOrder[3] = B1;
		break;
	case 2:
		stairOrder[0] = B0;
		stairOrder[1] = A1;
		stairOrder[2] = B1;
		stairOrder[3] = A0;
		break;

	case 3:
		stairOrder[0] = B1;
		stairOrder[1] = A0;
		stairOrder[2] = B0;
		stairOrder[3] = A1;
		break;
	}


	// trialFile directory
	string dirName = experiment_directory + subjectName;
	mkdir(dirName.c_str()); // windows syntax


	string responseFileName = dirName + "/" + subjectName + "_p" + session + "_match.txt";
	// Principal streams files
	if (util::fileExists(dirName + "/" + subjectName + "_p" + session + "_match.txt") && subjectName != "junk")
	{
		string error_on_file_io = string("file already exists");
		cerr << error_on_file_io << endl;
		MessageBox(NULL, (LPCSTR)"FILE ALREADY EXISTS\n Please check the parameters file.", NULL, NULL);
		shutdown();
	}

	responseFile.open(responseFileName.c_str());
	responseFile << fixed << responseFile_headers << endl;

	//globalTimer.start();
}


//Central function for projecting image onto the screen
void drawGLScene()
{
	glDrawBuffer(GL_BACK_RIGHT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);

	cam.setEye(eyeRight);
	drawStimulus();
	drawInfo();


	// Draw right eye view
	glDrawBuffer(GL_BACK_LEFT);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0, 0.0, 0.0, 1.0);

	cam.setEye(eyeLeft);
	//drawBlurred();
	drawStimulus();
	drawInfo();


	glutSwapBuffers();
}

void update(int value)
{
	glutPostRedisplay();
	glutTimerFunc(TIMER_MS, update, 0);
}

void drawProgressBar() {

	glPushMatrix();
	glLoadIdentity();
	glTranslated(0, 0, display_distance);

	glColor3f(0.2, 0.2, 0.2);
	glBegin(GL_LINE_LOOP);
	glVertex3f(-50, 5, 0);
	glVertex3f(50, 5, 0);
	glVertex3f(50, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();

	glColor3f(0.1, 0.3, 0.1);
	glBegin(GL_POLYGON);
	glVertex3f(-50, 5, 0);
	glVertex3f(-50 + percentComplete, 5, 0);
	glVertex3f(-50 + percentComplete, -5, 0);
	glVertex3f(-50, -5, 0);
	glEnd();
	glPopMatrix();
}


void drawInfo()
{
	// displays relevant information to the screen
	if (visibleInfo)
	{
		glDisable(GL_COLOR_MATERIAL);
		glDisable(GL_BLEND);
		GLText text;
		if (gameMode)
			text.init(SCREEN_WIDTH, SCREEN_HEIGHT, glWhite, GLUT_BITMAP_HELVETICA_18);
		else
			text.init(640, 480, glWhite, GLUT_BITMAP_HELVETICA_12);


		text.enterTextInputMode();

		switch (current_stage) {
		case stimulus_preview:
			glColor3fv(glWhite);
			text.draw("Welcome! press + to start training");
			text.draw("# Name: " + subjectName);
			text.draw("# IOD: " + stringify<double>(interoculardistance));
			if(testVisualStimuliOnly){
				text.draw("# depth texture: " + stringify<double>(depth_std_text));
				text.draw("# depth stereo: " + stringify<double>(depth_std_disp));
			}

			text.draw("                           ");

			if (abs(mirrorAlignment - 45.0) > 0.2)
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("# !!!!Mirror Alignment = " + stringify<double>(mirrorAlignment));

			text.draw("                           ");
			if (!screenDistanceCorrect)
				glColor3fv(glRed);
			else
				glColor3fv(glGreen);
			text.draw("screen1 " + stringify< Eigen::Matrix<double, 1, 3> >(markers[screen1].p.transpose()));


			break;

		case trial_respond:

			glColor3fv(glRed);
			text.draw("# Name: " + subjectName);
			text.draw("# IOD: " + stringify<double>(interoculardistance));
			text.draw("# block N: " + stringify<int>(blkNum));
			text.draw("# trial: " + stringify<int>(trialNum));
			//text.draw("# time: " + stringify<double>(ElapsedTime));
			//text.draw("# STD depth texture: " + stringify<double>(depth_std_text));
			//text.draw("# STD depth stereo: " + stringify<double>(depth_std_disp));
			text.draw("                           ");
			text.draw("# CMP depth: " + stringify<double>(depth_cmp));

			// check if mirror is calibrated				
			text.draw("# !!!!Mirror Alignment = " + stringify<double>(mirrorAlignment));

			break;

		case break_time:
			glColor3fv(glWhite);
			if (training && trialNum >= trainNum_cap) {
				text.draw("Pls call the experimenter!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
			}
			else {
				text.draw("Break time! Press + to continue");

				if (abs(mirrorAlignment - 45.0) > 0.2)
					glColor3fv(glRed);
				else
					glColor3fv(glGreen);
				text.draw("# !!!!Mirror Alignment = " + stringify<double>(mirrorAlignment));
			}
			break;

		case exp_completed:
			glColor3fv(glWhite);
			text.draw("The experiment is over. Thank you! :)");
			break;
		}
		text.leaveTextInputMode();
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_BLEND);
	}
}


// Funzione che gestisce il ridimensionamento della finestra
void handleResize(int w, int h)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glViewport(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

}

void drawStimulus()
{
	//enum Stages {stimulus_preview, prep_trial, trial_fixate_first, trial_present_first,
	//trial_fixate_second, trial_present_second, trial_respond, break_time, exp_completed};
	switch (current_stage) {

	case stimulus_preview:
		drawVertices(true, dist_std_toEye);

		break;

	case trial_fixate_first:
		if (std_is_first) {
			drawFixation(display_distance_jittered_std);
		}
		else {
			drawFixation(display_distance_jittered_cmp);
		}
		break;

	case trial_present_first:
		if (std_is_first)
			drawVertices(true, dist_std_toEye);
		else
			drawVertices(false, dist_cmp_toEye);

		break;

	case trial_fixate_second:
		if (std_is_first) {
			drawFixation(display_distance_jittered_cmp);
		}
		else {
			drawFixation(display_distance_jittered_std);
		}
		break;

	case trial_present_second:
		if (std_is_first)
			drawVertices(false, dist_cmp_toEye);
		else
			drawVertices(true, dist_std_toEye);
		break;

	case break_time:
		drawProgressBar();
		break;

	}
}




void initTrial()
{
	current_stage = prep_trial;
	initProjectionScreen(display_distance);

	if (rand() % 2) {
		std_is_first = true;
	}
	else {
		std_is_first = false;
	}

	if (training) {

		depth_std = rand() % depth_training_range + depth_training_min;
		depth_std_disp = depth_std;
		depth_std_text = depth_std;
		depth_cmp = rand() % depth_training_range + depth_training_min;
	}
	else {
		stairID = blkNum - 1;
		//stairID = trial.getCurrent().second->getCurrentStaircase()->getID();
		ascending = trial.getCurrent().second->getCurrentStaircase()->getAscending();
		stair_reversal = trial.getCurrent().second->getCurrentStaircase()->getReversals();
		

		depth_std = trial.getCurrent().first["stdDepth"];
		depth_stdDelta = trial.getCurrent().first["stdDepthDelta"];
		depth_std_text = depth_std + depth_stdDelta;
		depth_std_disp = depth_std - depth_stdDelta;

		depth_cmp = trial.getCurrent().second->getCurrentStaircase()->getState();
	}

	jitter_z_std = ((rand() % 41) - 20) / 2.0; // from -10 to 10
	jitter_z_cmp = ((rand() % 41) - 20) / 2.0; // from -10 to 10

	initStimulus_std();
	initStimulus_cmp();

	trial_timer.reset();
	trial_timer.start();
	ElapsedTime = 0;
	TimerFrameCnt = 0;
	last_timer_frame_cnt = 0;

	current_stage = trial_fixate_first;
}

void onlineTrial() {
	//stimulus_preview, prep_trial, trial_fixate_first, trial_present_first, trial_fixate_second, trial_present_second, trial_respond,

	switch (current_stage) {

	case stimulus_preview:
	case break_time:
		if(!testVisualStimuliOnly)
			check_apparatus_alignment();
		break;

	case trial_fixate_first:

		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();
		//TimerFrameCnt++;

		if (ElapsedTime > fixateTime) {
			//if (TimerFrameCnt > fixateFrameCnt) {
			beepOk(19);
			lastTimeStamp = trial_timer.getElapsedTimeInMilliSec();
			//last_timer_frame_cnt = TimerFrameCnt;
			current_stage = trial_present_first;

		}
		break;

	case trial_present_first:

		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();


		if ((ElapsedTime - lastTimeStamp) > (viewTime)) {

			lastTimeStamp = trial_timer.getElapsedTimeInMilliSec();

			current_stage = trial_fixate_second;
		}
		break;

	case trial_fixate_second:

		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();


		if ((ElapsedTime - lastTimeStamp) > (fixateTime)) {

			beepOk(20);
			lastTimeStamp = trial_timer.getElapsedTimeInMilliSec();

			current_stage = trial_present_second;
		}
		break;

	case trial_present_second:

		ElapsedTime = trial_timer.getElapsedTimeInMilliSec();


		if ((ElapsedTime - lastTimeStamp) > (viewTime)) {

			lastTimeStamp = trial_timer.getElapsedTimeInMilliSec();

			current_stage = trial_respond;
		}
		break;
	}

}

void advanceTrial()
{
	//subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tclyHorizontal\ttexnum\ttextNomralizer\ttestDepth\tprobeStart\tprobeDepth\ttime
	if (training) {
		if (trialNum < trainNum_cap) {
			beepOk(4);
			trialNum++;
			initTrial();
		}
		else {
			beepOk(2);
			trialNum++;
			current_stage = break_time;
			visibleInfo = true;
		}

	}
	else {
		//subjName\tIOD\tblockN\ttrialN\tdisplayDistance\tvisualAngle\tshapeID\ttexnum\ttextNomralizer\ttestDepth\tprobeDepthInit\tprobeDepth\tRT
		responseFile << fixed <<
			subjectName << "\t" <<
			interoculardistance << "\t" <<
			blkNum << "\t" <<
			trialNum << "\t" <<
			depth_std_text << "\t" <<
			depth_std_disp << "\t" <<
			depth_cmp << "\t" <<
			std_is_first << "\t" <<
			respond_first_deeper << "\t" <<
			respond_cmp_deeper << "\t" <<
			responseTime << "\t" <<
			stairID << "\t" <<
			stair_reversal << "\t" <<
			ascending << "\t" <<
			display_distance_jittered_std << "\t" <<
			display_distance_jittered_cmp << "\t" <<	
			visual_angle << "\t" <<
			Tex_dot_density << "\t" <<
			Tex_dot_radius << "\t" <<
			Tex_dot_separation_ratio
			<< endl;

		trial.next(respond_cmp_deeper);

		
		if (!trial.isEmpty()) {

			if (trialNum % 20 == 0) {
				beepOk(4);
				percentComplete = trialNum / (totalTrNum / 100.0);
				current_stage = break_time;
				visibleInfo = true;

			}
			else {
				beepOk(4);
				trialNum++;
				initTrial();
			}

		}
		else if (blkNum < totalBlkNum) {
			beepOk(4);
			blkNum++;
			initBlock(blkNum);
			initTrial();
		}
		else {
			beepOk(1);
			responseFile.close();
			visibleInfo = true;
			current_stage = exp_completed;
		}

		/*
		if(testVisualStimuliOnly){
			if (trialNum<2) {

				if (trialNum % 20 == 0) {
					beepOk(4);
					percentComplete = trialNum / (totalTrNum / 100.0);
					current_stage = break_time;
					visibleInfo = true;

				}
				else {
					beepOk(4);
					trialNum++;
					initTrial();
				}

			}
			else if (blkNum < totalBlkNum) {
				blkNum++;
				initBlock(blkNum);
				trialNum = 1;
				initTrial();
			}
			else {
				beepOk(1);
				responseFile.close();
				visibleInfo = true;
				current_stage = exp_completed;
			}

		}
		*/
	

	}


}

void handleKeypress(unsigned char key, int x, int y)
{
	switch (key) { // key presses that work regardless of the stage

	case 27:	//corrisponde al tasto ESC
		shutdown();
		break;

	case 'i':
		visibleInfo = !visibleInfo;
		break;


	case '1':
		
		if (current_stage == stimulus_preview) {
			if (depth_std_text > depth_inc) {
				depth_std_text = depth_std_text - depth_inc;
				depth_std_disp = depth_std_text;
			}
			initStimulus_std();


		}
		else if (current_stage == trial_respond) {

			responseTime = trial_timer.getElapsedTimeInMilliSec() - lastTimeStamp;
			trial_timer.stop();
			respond_first_deeper = true;
			if (std_is_first) {
				respond_cmp_deeper = false;
			}
			else {
				respond_cmp_deeper = true;
			}

			advanceTrial();
		}

		break;

	case '2':
	
		if (current_stage == stimulus_preview) {

			depth_std_text = depth_std_text + depth_inc;
			depth_std_disp = depth_std_text;
			initStimulus_std();

		}
		else if (current_stage == trial_respond) {

			responseTime = trial_timer.getElapsedTimeInMilliSec() - lastTimeStamp;
			trial_timer.stop();

			respond_first_deeper = false;
			if (std_is_first) {
				respond_cmp_deeper = true;
			}
			else {
				respond_cmp_deeper = false;
			}

			advanceTrial();
		}

		break;

		
	case '+':
		switch (current_stage) {
		case stimulus_preview:
			if ((abs(mirrorAlignment - 45.0) < 0.2) && screenDistanceCorrect) {
				beepOk(5);
				visibleInfo = false;
				initBlock(blkNum);
				initTrial();
			}
			else {
				beepOk(3);
			}

			break;

		case break_time:
			if (abs(mirrorAlignment - 45.0) < 0.2) {
				beepOk(5);
				visibleInfo = false;
				trialNum++;
				initTrial();
			}
			else {
				beepOk(8);
			}
			break;
		}
		break;

	case 'T':
	case 't':
		if (training) {
			training = false;
			visibleInfo = true;
			beepOk(6);
			trialNum = 0;
			current_stage = break_time;
		}
		break;

		///////////////////////////////////////////////
		// key presses for adjusting stimulus previe
	case '4':
	
		if (current_stage == stimulus_preview) {

			if (depth_std_text > depth_inc)
				depth_std_text = depth_std_text - depth_inc;

			initStimulus_std();

		}

		break;

	case '5':

		if (current_stage == stimulus_preview) {

			depth_std_text = depth_std_text + depth_inc;

			initStimulus_std();

		}

		break;


	case '7':
		if (current_stage == stimulus_preview) {

			if (depth_std_disp > depth_inc)
				depth_std_disp = depth_std_disp - depth_inc;

			initStimulus_std();

		}
		break;

	case '8':
		if (current_stage == stimulus_preview) {

			depth_std_disp = depth_std_disp + depth_inc;

			initStimulus_std();

		}
		break;


	case 'a':
		initTrial();
		break;

	}

}



void idle()
{
	onlineTrial();

}

/*** Online operations ***/
void check_apparatus_alignment()
{
	updateTheMarkers();
	// mirror alignment check
	if (isVisible(markers.at(mirror1).p) && isVisible(markers.at(mirror2).p)) {
		mirrorAlignment = asin(
			abs((markers.at(mirror1).p.z() - markers.at(mirror2).p.z())) /
			sqrt(
				pow(markers.at(mirror1).p.x() - markers.at(mirror2).p.x(), 2) +
				pow(markers.at(mirror1).p.z() - markers.at(mirror2).p.z(), 2)
			)
		) * 180 / M_PI;
	}
	else {
		mirrorAlignment = 999;//999;
	}

	// screen Y alignment check
	if (isVisible(markers.at(screen1).p) && isVisible(markers.at(screen3).p)) {
		screenAlignmentY = asin(
			abs((markers.at(screen1).p.y() - markers.at(screen3).p.y())) /
			sqrt(
				pow(markers.at(screen1).p.x() - markers.at(screen3).p.x(), 2) +
				pow(markers.at(screen1).p.y() - markers.at(screen3).p.y(), 2)
			)
		) * 180 / M_PI;
	}
	else {
		screenAlignmentY = 999;
	}

	// screen Z alignment check
	if (isVisible(markers.at(screen1).p) && isVisible(markers.at(screen2).p)) {
		screenAlignmentZ = asin(
			abs(markers.at(screen1).p.z() - markers.at(screen2).p.z()) /
			sqrt(
				pow(markers.at(screen1).p.x() - markers.at(screen2).p.x(), 2) +
				pow(markers.at(screen1).p.z() - markers.at(screen2).p.z(), 2)
			)
		) * 180 / M_PI *
			abs(markers.at(screen1).p.x() - markers.at(screen2).p.x()) /
			(markers.at(screen1).p.x() - markers.at(screen2).p.x());
	}
	else {
		screenAlignmentZ = 999;
	}

	if (abs(markers[screen1].p.x() - (-330.0)) > 10) {
		screenDistanceCorrect = false;
	}
	else {
		screenDistanceCorrect = true;
	}
}


void initStimulus_std() {

	display_distance_jittered_std = display_distance + jitter_z_std;

	//dist_std_toEye = -(display_distance_jittered_std -  (depth_std_text + depth_std_disp) / 2.0);
	dist_std_toEye = -(display_distance_jittered_std - depth_std_disp);

	stimulus_height_std = tan((DEG2RAD * visual_angle) / 2) * 2 * dist_std_toEye;
	stimulus_visiblewidth_std = ratio_visiblewidth_height * stimulus_height_std;
	
	if (abs(depth_std_text - depth_std_disp) < 0.1) {
		buildSurface_congruent(stimulus_width, stimulus_height_std, depth_std_text, dist_std_toEye, my_vertices_data_std, stimulus_visiblewidth_std, my_contour_data_std);
	}
	else {
		buildSurface_incongruent(stimulus_width, stimulus_height_std, depth_std_disp, depth_std_text, dist_std_toEye, my_vertices_data_std, stimulus_visiblewidth_std, my_contour_data_std);
	}

	// Light source parameters
	float temp_max_intensity = max_intensity_flat_light + (max_intensity_deep_light - max_intensity_flat_light) * (depth_std_text - depth_flat_light) / (depth_deep_light - depth_flat_light);
	amb_intensity_std = adjustAmbient(depth_std_text, temp_max_intensity, ambVDif_flat_light, ambVDif_deep_light, depth_flat_light, depth_deep_light);
	LightAmbient_std[0] = amb_intensity_std; // non-directional & overall light (r,g,b,alpha): dark part
	LightDiffuse_std[0] = temp_max_intensity - amb_intensity_std;


}


void initStimulus_cmp() {

	display_distance_jittered_cmp = display_distance + jitter_z_cmp;
	dist_cmp_toEye = -(display_distance_jittered_cmp - depth_cmp);
	stimulus_height_cmp = tan((DEG2RAD * visual_angle) / 2) * 2 * dist_cmp_toEye;
	stimulus_visiblewidth_cmp = ratio_visiblewidth_height * stimulus_height_cmp;
	buildSurface_congruent(stimulus_width, stimulus_height_cmp, depth_cmp, dist_cmp_toEye, my_vertices_data_cmp, stimulus_visiblewidth_cmp, my_contour_data_cmp);
	float temp_max_intensity = max_intensity_flat_light + (max_intensity_deep_light - max_intensity_flat_light) * (depth_cmp - depth_flat_light) / (depth_deep_light - depth_flat_light);
	amb_intensity_cmp = adjustAmbient(depth_cmp, temp_max_intensity, ambVDif_flat_light, ambVDif_deep_light, depth_flat_light, depth_deep_light);
	LightAmbient_cmp[0] = amb_intensity_cmp; // non-directional & overall light (r,g,b,alpha): dark part
	LightDiffuse_cmp[0] = temp_max_intensity - amb_intensity_cmp;

}




// this is run at compilation because it's titled 'main'
int main(int argc, char* argv[])
{
	//functions from cncsvision packages
	mathcommon::randomizeStart();

	// initializing glut (to use OpenGL)
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_STEREO);

	glutGameModeString("1024x768:32@85"); //resolution  
	glutEnterGameMode();
	glutFullScreen();

	// initializes optotrak and velmex motors
	if(!testVisualStimuliOnly){
		
		initOptotrak();
	}
	initMotors();

	initRendering(); // initializes the openGL parameters needed for creating the stimuli

	//glewInit();
	//initBlur();

	initStreams(); // streams as in files for writing data

	initVariables();

	initStimulus_std();

	initProjectionScreen(display_distance);

	// glut callback, OpenGL functions that are infinite loops to constantly run 

	glutDisplayFunc(drawGLScene); // keep drawing the stimuli

	glutKeyboardFunc(handleKeypress); // check for keypress

	glutReshapeFunc(handleResize);

	glutIdleFunc(idle);

	glutTimerFunc(TIMER_MS, update, 0);

	glutSetCursor(GLUT_CURSOR_NONE);

	//boost::thread initVariablesThread(&initVariables); 

	glutMainLoop();

	//cleanup();
	return 0;
}

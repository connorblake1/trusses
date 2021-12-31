#include "Joint.cpp"
#include "Platform/Platform.hpp"
Joint* truss;
Joint* vtruss;
int regions = Joint::REGIONS;
float l = Joint::RLENGTH;
float startCarX = 10;
float carX = startCarX;
float carStep = .1;
float carLoad = 1000;
float small = .0001;
float jr = 10;
float Young = 29000;
float A = 2;
const float rat = sqrt(3) / 2;
float winx, winy, sx, sy;
sf::RectangleShape* trusses;
sf::CircleShape* joints;
sf::RectangleShape car(sf::Vector2f(20, 10));
sf::RectangleShape arr1(sf::Vector2f(10, 20));
sf::RectangleShape arr2(sf::Vector2f(10, 20));
sf::RectangleShape pin(sf::Vector2f(2 * jr, 2 * jr));
sf::CircleShape roller;

//warren truss
const int NUMJ = regions * 2 + 1;
const int NUMT = regions * 4 - 1;

bool* init; // = new bool[NUMT];
bool* vinit;
float* spans; // = new float[NUMT];
float* vspans;

void drawTruss(sf::RenderWindow* wind);
bool inBounds(int n);
void solveMethodOfJoints(Joint* gt, bool* crosser, float* loads);
void solveVirtualWork();
bool unbalanced(Joint* gt);
bool approx(float in);
int spanIndex(int j1, int j2);
float distS(int j1, int j2, Joint* gt);
float distR(int j1, int j2, Joint* gt);
float theta(int j1, int j2, Joint* gt);
int map(float in, float l1, float u1, float l2, float u2);
void initializeVars();
void initializeVVars();
void resetVT();
void resetVJ();

void resetVJ()
{
	for (int i = 0; i < NUMJ; i++)
	{
		vtruss[i].lx = 0;
		vtruss[i].ly = 0;
	}
}
void resetVT()
{
	for (int i = 0; i < NUMT; i++)
	{
		vspans[i] = 0;
		vinit[i] = false;
	}
}

void initializeVars()
{
	for (int i = 0; i < NUMJ; i++)
	{
		truss[i].lx = 0;
		truss[i].ly = 0;
		truss[i].dx = 0;
		truss[i].dy = 0;
	}
	init = new bool[NUMT];
	spans = new float[NUMT];
	for (int i = 0; i < NUMT; i++)
	{
		spans[i] = 0;
		init[i] = false;
	}
}

void initializeVVars()
{
	for (int i = 0; i < NUMJ; i++)
	{
		vtruss[i].lx = 0;
		vtruss[i].ly = 0;
		vtruss[i].dx = 0;
		vtruss[i].dy = 0;
	}
	vinit = new bool[NUMT];
	vspans = new float[NUMT];
	for (int i = 0; i < NUMT; i++)
	{
		vspans[i] = 0;
		vinit[i] = false;
	}
}

int map(float in, float l1, float u1, float l2, float u2)
{
	return l2 + (u2 - l2) * (in - l1) / (u1 - l1);
}

float distS(int j1, int j2, Joint* gt)
{
	return sqrt(pow(gt[j1].x - gt[j2].x, 2) + pow(gt[j1].y - gt[j2].y, 2));
}
float distR(int j1, int j2, Joint* gt)
{
	return sqrt(pow(gt[j1].dx - gt[j2].dx, 2) + pow(gt[j1].dy - gt[j2].dy, 2));
}
float theta(int j1, int j2, Joint* gt)
{
	return atan2(gt[j2].y - gt[j1].y, gt[j2].x - gt[j1].x);
}

bool approx(float in)
{
	return (abs(in) < small);
}
bool inBounds(int n)
{
	return (n >= 0 && n < NUMJ);
}
void drawTruss(sf::RenderWindow* wind)
{

	for (int i = 0; i < NUMJ; i++)
	{
		joints[i].setPosition(sf::Vector2f(truss[i].x + sx + truss[i].dx, truss[i].y + sy + truss[i].dy));
	}
	for (int i = 0; i < NUMT; i++)
	{
		//std::cout << i << " " << spans[i] << std::endl;
		int i1 = -1;
		int i2 = -1;
		if (i < regions)
		{
			i1 = i;
			i2 = i + 1;
		}
		else if (i >= regions && i < 3 * regions)
		{
			i1 = (i + 1 - regions) / 2;
			i2 = (i - regions) / 2 + regions + 1;
		}
		else
		{
			i1 = i - 2 * regions + 1;
			i2 = i - 2 * regions + 2;
		}
		float x1 = joints[i1].getPosition().x;
		float y1 = joints[i1].getPosition().y;
		float x2 = joints[i2].getPosition().x;
		float y2 = joints[i2].getPosition().y;
		float li = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
		float ti = atan2(y2 - y1, x2 - x1);
		trusses[i].setSize(sf::Vector2f(li, map(abs(spans[i]), 0, 2000, 5, jr * 2)));
		if (spans[i] < 0)
		{
			trusses[i].setFillColor(sf::Color(map(spans[i], 0, -2000, 50, 255), 0, 0));
		}
		else
		{
			trusses[i].setFillColor(sf::Color(0, 0, map(spans[i], 0, 2000, 50, 255)));
		}
		trusses[i].setPosition(sf::Vector2f(x1, y1));
		trusses[i].setRotation(180 / 3.1415 * ti);
		wind->draw(trusses[i]);
	}

	for (int i = 0; i < NUMJ; i++)
	{
		wind->draw(joints[i]);
	}
	int i1 = floor(carX / l);
	float modCar = carX - l * i1;
	int i2 = i1 + 1;
	float interp = map(modCar, 0, l, joints[i1].getPosition().y, joints[i2].getPosition().y);
	car.setPosition(carX + sx, interp);
	wind->draw(car);
	// wind->draw(arr1);
	// wind->draw(arr2);
	wind->draw(roller);
	wind->draw(pin);
}

int spanIndex(int j1, int j2)
{
	if (j1 <= regions && j2 <= regions)
	{
		return fmin(j1, j2);
	}
	else if (j1 <= regions && j2 > regions)
	{
		return j2 - 1 + j1;
	}
	else if (j1 > regions && j2 <= regions)
	{
		return j1 - 1 + j2;
	}
	return fmin(j1, j2) + 2 * regions - 1;
}

bool unbalanced(Joint* gt)
{
	for (int i = 0; i < NUMJ; i++)
	{
		if (abs(gt[i].lx) > small || abs(gt[i].ly) > small)
		{
			return true;
		}
	}
	return false;
}

void solveMethodOfJoints(Joint* gt, bool* crosser, float* loads)
{
	//std::cout << "Solving Method of Joints" << std::endl;
	for (int i = 0; i < NUMJ; i++)
	{
		gt[i].solved = 0;
		//std::cout << "Forces " << i << ": " << gt[i].lx << " " << gt[i].ly << std::endl;
	}
	//int ticker = 0;
	while (unbalanced(gt))
	{
		// ticker++;
		// if (ticker > 15) {return;}
		int ind = -1;
		//pick joint to solve with only 2 unknowns
		for (int i = 0; i < NUMJ; i++)
		{
			if (gt[i].js - gt[i].solved == 2)
			{
				ind = i;
				break;
			}
			else if (gt[i].js - gt[i].solved == 1)
			{
				// for (int i = 0; i < NUMJ; i++)
				// {
				// 	std::cout << "Forces on " << i << ": " << gt[i].lx << " " << gt[i].ly << std::endl;
				// }
				for (int j = 0; j < gt[i].js; j++)
				{
					int thisSpan = spanIndex(i, gt[i].joints[j]);
					if (!crosser[thisSpan])
					{
						float ttheta = -theta(i, gt[i].joints[j], gt);
						float li = gt[i].lx / cos(ttheta);

						loads[thisSpan] = li;
						crosser[thisSpan] = true;
						gt[i].solved += 1;
						gt[gt[i].joints[j]].solved += 1;
						gt[i].lx -= li * cos(ttheta);
						gt[i].ly -= li * sin(ttheta);
						gt[gt[i].joints[j]].lx += li * cos(ttheta);
						gt[gt[i].joints[j]].ly += li * sin(ttheta);
						break;
					}
				}
			}
		}
		//std::cout << "\t\t" << ind << std::endl;
		if (!unbalanced(gt))
		{
			break;
		}
		if (ind == -1)
		{
			break;
		}
		//std::cout << "current solving joint: " << ind << " \t Forces: " << gt[ind].lx << " " << gt[ind].ly << std::endl;

		// for (int i = 0; i < NUMJ; i++)
		// {
		// 	std::cout << "Forces on " << i << ": " << gt[i].lx << " " << gt[i].ly << std::endl;
		// }
		//figure out the missing y load

		float l[2] = { 0, 0 };
		int ii[2] = { -1, -1 };
		int filled = 0;
		float ttheta[2] = { 0, 0 };
		for (int i = 0; i < gt[ind].js; i++)
		{
			int thisSpan = spanIndex(ind, gt[ind].joints[i]);
			if (!crosser[thisSpan])
			{
				ii[filled] = i;
				filled++;
				if (filled == 2)
				{
					break;
				}
			}
		}
		float m = 0;
		ttheta[0] = -theta(ind, gt[ind].joints[ii[0]], gt);
		ttheta[1] = -theta(ind, gt[ind].joints[ii[1]], gt);
		float thing = 0;
		if (gt[ind].lx == 0)
		{
			thing = -cos(ttheta[1]) / cos(ttheta[0]);
			l[1] = gt[ind].ly / (sin(ttheta[1]) + thing * sin(ttheta[0]));
			l[0] = l[1] * thing;
			//std::cout << "Thing 1: " << thing << std::endl;
		}
		else if (gt[ind].ly == 0)
		{
			thing = -sin(ttheta[1]) / sin(ttheta[0]);
			l[1] = gt[ind].lx / (cos(ttheta[1]) + thing * cos(ttheta[0]));
			l[0] = l[1] * thing;
		}
		else
		{
			m = gt[ind].ly / gt[ind].lx;
			thing = (sin(ttheta[1]) - m * cos(ttheta[1])) / (m * cos(ttheta[0]) - sin(ttheta[0]));
			l[0] = gt[ind].lx / (cos(ttheta[0]) + cos(ttheta[1]) / thing);
			l[1] = l[0] / thing;
			//std::cout << "Thing 2: " << thing << std::endl;
		}

		int tsi[2] = { spanIndex(ind, gt[ind].joints[ii[0]]), spanIndex(ind, gt[ind].joints[ii[1]]) };
		loads[tsi[0]] = l[0];
		loads[tsi[1]] = l[1];
		crosser[tsi[0]] = true;
		crosser[tsi[1]] = true;
		gt[ind].solved += 2;
		gt[gt[ind].joints[ii[0]]].solved += 1;
		gt[gt[ind].joints[ii[1]]].solved += 1;
		gt[ind].lx -= l[0] * cos(ttheta[0]) + l[1] * cos(ttheta[1]);
		gt[ind].ly -= l[0] * sin(ttheta[0]) + l[1] * sin(ttheta[1]);
		gt[gt[ind].joints[ii[0]]].lx += l[0] * cos(ttheta[0]);
		gt[gt[ind].joints[ii[0]]].ly += l[0] * sin(ttheta[0]);
		gt[gt[ind].joints[ii[1]]].lx += l[1] * cos(ttheta[1]);
		gt[gt[ind].joints[ii[1]]].ly += l[1] * sin(ttheta[1]);
		// std::cout << "New Angles: " << ttheta[0] << " " << ttheta[1] << std::endl;
		// std::cout << "New Forces: " << l[0] << " " << l[1] << std::endl;

		// float t1 = 0;
		// float tTheta = 0;
		// for (int i = 0; i < gt[ind].js; i++)
		// {
		// 	tTheta = -theta(ind,gt[ind].joints[i]);
		// 	int thisSpan = spanIndex(ind, gt[ind].joints[i]);
		// 	if (!crosser[thisSpan] && !(approx(tTheta) || approx(tTheta - 3.1415) || approx(tTheta + 3.1415)))
		// 	{
		// 		t1 = gt[ind].ly / sin(tTheta);
		// 		std::cout << "tTheta " << tTheta << " " << "   t1 " << t1 << std::endl;

		// 		loads[thisSpan] = t1;
		// 		crosser[thisSpan] = true;
		// 		gt[ind].solved += 1;
		// 		gt[gt[ind].joints[i]].solved += 1;
		// 		gt[ind].lx -= t1 * cos(tTheta);
		// 		gt[ind].ly -= t1 * sin(tTheta);
		// 		gt[gt[ind].joints[i]].lx += t1 * cos(tTheta);
		// 		gt[gt[ind].joints[i]].ly += t1 * sin(tTheta);

		// 		break;
		// 	}
		// }
		// std::cout << "Current Forces on " << ind << ": " << gt[ind].lx << " " << gt[ind].ly << std::endl;
		// std::cout << "loads1: " << std::endl;
		// for (int i = 0; i < NUMT; i++)
		// {
		// 	std::cout << loads[i] << " " << crosser[i] << std::endl;
		// }
		// std::cout << "Joints Solved1: " << std::endl;
		// for (int i = 0; i < NUMJ; i++)
		// {
		// 	std::cout << gt[i].js << " " << gt[i].solved << std::endl;
		// 	if (gt[i].js == gt[i].solved)
		// 	{
		// 		std::cout << "\t" << gt[i].lx << " " << gt[i].ly << std::endl;
		// 	}
		// }

		// //figure out the missing x load
		// for (int i = 0; i < gt[ind].js; i++)
		// {
		// 	float jTheta = -theta(ind,gt[ind].joints[i]);
		// 	int thisSpan = spanIndex(ind, gt[ind].joints[i]);
		// 	if (!crosser[thisSpan] && (approx(jTheta) || approx(jTheta - 3.1415) || approx(jTheta + 3.1415)))
		// 	{
		// 		float t2 = (gt[ind].lx / cos(jTheta));
		// 		std::cout << "jTheta " << jTheta << " " << "   t2 " << t2 << std::endl;

		// 		loads[thisSpan] = t2;
		// 		crosser[thisSpan] = true;
		// 		gt[ind].solved += 1;
		// 		gt[gt[ind].joints[i]].solved += 1;
		// 		gt[ind].lx -= t2 * cos(jTheta);
		// 		gt[ind].ly -= t2 * sin(jTheta);
		// 		gt[gt[ind].joints[i]].lx += t2 * cos(jTheta);
		// 		gt[gt[ind].joints[i]].ly += t2 * sin(jTheta);
		// 	}
		// }

		//printer section
		// std::cout << "loads2: " << std::endl;
		// for (int i = 0; i < NUMT; i++)
		// {
		// 	std::cout << loads[i] << " " << crosser[i] << std::endl;
		// }
		// std::cout << "Joints Solved2: " << std::endl;
		// for (int i = 0; i < NUMJ; i++)
		// {
		// 	std::cout << gt[i].js << " " << gt[i].solved << std::endl;
		// 	if (gt[i].js == gt[i].solved)
		// 	{
		// 		std::cout << "\t" << gt[i].lx << " " << gt[i].ly << std::endl;
		// 	}
		// }
	}
	// for (int i = 0; i < NUMT; i++)
	// {
	// 	std::cout << "index " << i << ": " << loads[i] << " " << crosser[i] << std::endl;
	// }
	// for (int i = 0; i < NUMJ; i++)
	// {
	// 	std::cout << "Forces on " << i << ": " << truss[i].lx << " " << truss[i].ly << std::endl;
	// }
}

void solveVirtualWork()
{
	for (int i = 0; i < NUMJ; i++)
	{
		initializeVVars();
		vtruss[i].ly += 1;
		float B = vtruss[i].x / regions / l;
		vtruss[regions].ly -= B;
		vtruss[0].ly -= B * (l * regions / vtruss[i].x - 1);
		// std::cout << "index iy: " << i << std::endl;
		// std::cout << "\t" << vtruss[i].ly << std::endl;
		// std::cout << "\t" << vtruss[regions].ly << std::endl;
		// std::cout << "\t" << vtruss[0].ly << std::endl;
		// std::cout << std::endl;
		solveMethodOfJoints(vtruss, vinit, vspans);
		float vsum = 0;
		for (int j = 0; j < NUMT; j++)
		{
			vsum += l * vspans[j] * spans[j];
		}
		truss[i].dy = -vsum / A / Young;

		initializeVVars();
		vtruss[i].lx += 1;
		vtruss[0].lx += -1;
		if (i > regions)
		{
			vtruss[0].ly = -rat / regions;
			vtruss[regions].ly = rat / regions;
		}
		// std::cout << "index ix: " << i << std::endl;
		// std::cout << "\t" << vtruss[i].lx << std::endl;
		// std::cout << "\t" << vtruss[0].lx << std::endl;
		// std::cout << "\t" << vtruss[0].ly << std::endl;
		// std::cout << "\t" << vtruss[regions].ly << std::endl;
		// std::cout << std::endl;
		solveMethodOfJoints(vtruss, vinit, vspans);
		vsum = 0;
		for (int j = 0; j < NUMT; j++)
		{
			vsum += l * vspans[j] * spans[j];
		}
		truss[i].dx = vsum / A / Young;
		//std::cout << "defection: " << i << " - " << truss[i].dy << " " << vsum << std::endl;
		// std::cout << "x: " << i << std::endl;
		// for (int j = 0; j < NUMT; j++)
		// {
		// 	std::cout << "\t" << vspans[j] << std::endl;
		// }
	}
}

int main()
{
	util::Platform platform;

#if defined(_DEBUG)
	std::cout << "Hello World!" << std::endl;
#endif
	sf::RenderWindow window;
	// in Windows at least, this must be called before creating the window
	float screenScalingFactor = platform.getScreenScalingFactor(window.getSystemHandle());
	// Use the screenScalingFactor
	winx = 800;
	winy = 800;
	window.create(sf::VideoMode(winx * screenScalingFactor, winy * screenScalingFactor), "SFML works!");
	platform.setIcon(window.getSystemHandle());
	Joint dummy = Joint();
	dummy.setX(5);

	// Joint::setRegions(regions);
	// Joint::setLength(l);

	//initialize all the connections and positions of WARREN TRUSS
	std::cout << "Setting Up Truss" << std::endl;

	truss = new Joint[NUMJ];
	vtruss = new Joint[NUMJ];
	//by convention, 0 is pin joint, REGIONS is roller
	for (int i = 0; i < NUMJ; i++)
	{
		truss[i] = Joint();
		vtruss[i] = Joint();
	}
	for (int i = 0; i < NUMJ; i++)
	{
		if (i <= regions)
		{
			truss[i].setX(i * l);
			truss[i].setY(0);
			vtruss[i].setX(i * l);
			vtruss[i].setY(0);
		}
		else
		{
			truss[i].setX(-l / 2 + (i - regions) * l);
			truss[i].setY(-l * rat);
			vtruss[i].setX(-l / 2 + (i - regions) * l);
			vtruss[i].setY(-l * rat);
		}
		// std::cout << "i: " << i<< std::endl;
		// std::cout << "Positions: " << truss[i].x << " " << truss[i].y << std::endl;
		if (i <= regions)
		{
			if (inBounds(i - 1))
			{
				truss[i].addJoint(i - 1);
				vtruss[i].addJoint(i - 1);
			}
			if (inBounds(regions + i) && i > 0)
			{
				truss[i].addJoint(regions + i);
				vtruss[i].addJoint(regions + i);
			}
			if (inBounds(regions + i + 1) && i != regions)
			{
				truss[i].addJoint(regions + i + 1);
				vtruss[i].addJoint(regions + i + 1);
			}
			if (inBounds(i + 1) && i != regions)
			{
				truss[i].addJoint(i + 1);
				vtruss[i].addJoint(i + 1);
			}
		}
		else
		{
			if (inBounds(i - 1) && i != regions + 1)
			{
				truss[i].addJoint(i - 1);
				vtruss[i].addJoint(i - 1);
			}
			truss[i].addJoint(i - regions - 1);
			truss[i].addJoint(i - regions);
			vtruss[i].addJoint(i - regions - 1);
			vtruss[i].addJoint(i - regions);
			if (inBounds(i + 1) && i != regions)
			{
				truss[i].addJoint(i + 1);
				vtruss[i].addJoint(i + 1);
			}
		}

		// std::cout << "connections: "<< std::endl;
		// for (int j = 0; j< truss[i].js; j++) {
		// 	std::cout << " \t\t " << truss[i].joints[j] << std::endl;
		// }
	}
	std::cout << "Truss Set Up." << std::endl;

	//initialize the shapes
	std::cout << "Making Shapes..." << std::endl;

	sx = 100;
	sy = 400;
	car.setFillColor(sf::Color::Green);
	car.setOrigin(0, 15);
	arr1.setFillColor(sf::Color::Green);
	arr2.setFillColor(sf::Color::Green);
	arr1.setRotation(240);
	arr2.setRotation(120);
	pin.setFillColor(sf::Color::Red);
	roller.setRadius(jr);
	roller.setFillColor(sf::Color::Red);
	pin.setPosition(sx - jr, sy + jr);
	roller.setPosition(sx + regions * l, sy + jr);
	joints = new sf::CircleShape[NUMJ];
	trusses = new sf::RectangleShape[NUMT];
	for (int i = 0; i < NUMJ; i++)
	{
		joints[i].setFillColor(sf::Color::White);
		joints[i].setRadius(jr);
		joints[i].setOrigin(jr, jr);
		joints[i].setPosition(truss[i].x + sx, truss[i].y + sy);
		//std::cout << "total connections: (" << i << ") " << truss[i].js << std::endl;
		for (int j = 0; j < truss[i].js; j++)
		{
			int ind = spanIndex(i, truss[i].joints[j]);
			//std::cout << "ind: " << ind << std::endl;
			trusses[ind].setFillColor(sf::Color(255, 255, 255));
			trusses[ind].setSize(sf::Vector2f(l, jr));
			trusses[ind].setOrigin(0, jr / 2);
			trusses[ind].setPosition(truss[i].x + sx, truss[i].y + sy);
			trusses[ind].setRotation(180 / 3.14 * theta(i, truss[i].joints[j], truss));
		}
	}
	std::cout << "Shapes Completed." << std::endl;

	std::cout << "Starting Loop" << std::endl;
	while (window.isOpen())
	{
		sf::Event event;
		while (window.pollEvent(event))
		{
			if (event.type == sf::Event::Closed)
			{
				window.close();
			}
		}

		window.clear();
		initializeVars();
		float reactB = carLoad * carX / regions / l;
		float reactA = reactB * (regions * l / carX - 1);
		truss[0].ly = reactA;
		truss[regions].ly = reactB;
		int i1 = floor(carX / l);
		float modCar = carX - l * i1;
		int i2 = i1 + 1;
		float B = carLoad * modCar / l;
		truss[i2].ly += -B;
		truss[i1].ly += -B * (l / modCar - 1);

		// float tUP = 0;
		// for (int i = 0; i <= regions; i++)
		// {
		// 	tUP += truss[i].ly;
		// }
		// std::cout << "Total" << tUP << std::endl;
		// std::cout << "Stresses" << truss[i1].ly << " " << truss[i2].ly << std::endl;
		solveMethodOfJoints(truss, init, spans);
		// std::cout << carX << " " << modCar << " " << carLoad << std::endl;
		// std::cout << "DONE" << std::endl;
		// for (int i = 0; i < NUMJ; i++)
		// {
		// 	std::cout << "Forces on " << i << ": " << truss[i].lx << " " << truss[i].ly << std::endl;
		// }

		solveVirtualWork();
		// arr1.setPosition(carX + sx, sy);
		// arr2.setPosition(carX + sx, sy);
		drawTruss(&window);

		window.display();
		carX += carStep;
		if (carX > l * regions)
		{
			carX = startCarX;
		}
	}

	return 0;
}

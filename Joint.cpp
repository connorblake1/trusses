class Joint
{
public:
	inline static int REGIONS = 5;
	inline static float RLENGTH = 150;
	inline static int MAXCONNECTIONS = 4;
	float x, y, dx, dy, lx, ly;
	bool fixX, fixY;
	int js;
	int solved;
	int* joints;
	//float * spans[MAXCONNECTIONS];
	float* il;

	void initialize(float xin, float yin, bool fxin, bool fyin)
	{
		this->x = xin;
		this->y = yin;
		this->fixX = fxin;
		this->fixY = fyin;
	}

	Joint()
	{
		js = 0;
		dx = 0;
		dy = 0;
		lx = 0;
		ly = 0;
		solved = 0;
		joints = new int[MAXCONNECTIONS];
		il = new float[REGIONS];
	}
	void setX(float xin)
	{
		this->x = xin;
	}
	void setY(float yin)
	{
		this->y = yin;
	}
	void setLX(float xin)
	{
		this->lx = xin;
	}
	void setLY(float yin)
	{
		this->ly = yin;
	}

	void addJoint(int jIndex)
	{
		joints[js] = jIndex;
		js++;
	}
	void setFixed(bool fxin, bool fyin)
	{
		this->fixX = fxin;
		this->fixY = fyin;
	}
};

#include <iostream>
#include <pthread.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#define _USE_MATH_DEFINES
#include <vector>
#include <math.h>

using namespace std;

int window_width=800;
int window_height=800;
int window_id=0;
const int fps=50;
const float pi = 3.1415926;

pthread_mutex_t mutex=PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t avatar_update_cond=PTHREAD_COND_INITIALIZER;

const int thread_num=1;
float axis[thread_num][3]={0,1.0f,0};
float translate[thread_num][3]={0.0f,0,0};
float color[thread_num][3]={1.0f,0,0};
float angle[thread_num]={0};
float angular_velocity[thread_num]={0};
bool update_avatar=false;
unsigned char avatar_key;

const float braking_force = 10.0f;
float anim_t = 0.0f;
float heading[3] = {1.0f, 0.0f, 0.0f};
float velocity[3] = {0.0f, 0.0f, 0.0f};
float location[3] = {0.0f, 0.0f, 0.0f};
vector<vector<float>> pos_ctl;

vector<float> vec_add(int dim, bool sub, vector<float> a, vector<float> b){
	vector<float> c(dim, 0);
	for(int i = 0; i < dim; i++){
		c[i] = a[i] + (sub? -1.0f * b[i] : b[i]);
	}
	return c;
}

vector<float> cubic_bezier(const float t,const vector<float>& point0,const vector<float>& point1,const vector<float>& point2,const vector<float>& point3)
{
	float b0=pow(1.0f-t,3);
	float b1=pow(1.0f-t,2)*t*3.0f;
	float b2=(1.0f-t)*t*t*3.0f;
	float b3=pow(t,3);

	vector<float> result(3,0);
	for(int i=0;i<3;i++){
		result[i]=b0*point0[i]+b1*point1[i]+b2*point2[i]+b3*point3[i];
	}

	return result;
}

float quaternion_magnitude_squared(const vector<float>& q)
{
	return pow(q[0],2)+pow(q[1],2)+pow(q[2],2)+pow(q[3],2);
}

vector<float> quaternion_inverse(const vector<float>& q)
{
	vector<float> q_conjugate=q;
	for(int i=1;i<4;i++){
		q_conjugate[i]*=-1.0f;
	}
	vector<float> result=q_conjugate;
	for(int i=0;i<4;i++){
		result[i]/=quaternion_magnitude_squared(q);
	}

	return result;
}

vector<float> quaternion_product(const vector<float>& q1,const vector<float>& q2)
{
	vector<float> result(4,0);
	result[0]=q1[0]*q2[0]-q1[1]*q2[1]-q1[2]*q2[2]-q1[3]*q2[3];
	result[1]=q1[0]*q2[1]+q1[1]*q2[0]+q1[2]*q2[3]-q1[3]*q2[2];
	result[2]=q1[0]*q2[2]-q1[1]*q2[3]+q1[2]*q2[0]+q1[3]*q2[1];
	result[3]=q1[0]*q2[3]+q1[1]*q2[2]-q1[2]*q2[1]+q1[3]*q2[0];

	return result;
}

vector<float> rotate_vector(const vector<float>& v,const vector<float>& q)
{
	//convert the vector v to quaternion form
	vector<float> q_v(4,0);
	for(int i=0;i<3;i++){
		q_v[i+1]=v[i];
	}
	vector<float> q_times_v=quaternion_product(q,q_v);
	vector<float> q_inv=quaternion_inverse(q);
	vector<float> q_result=quaternion_product(q_times_v,q_inv);
	vector<float> result(3,0);
	for(int i=0;i<3;i++){
		result[i]=q_result[i+1];
	}
	return result;
}

float quaternion_dot_product(const vector<float>& q1,const vector<float>& q2)
{
	return q1[0]*q2[0]+q1[1]*q2[1]+q1[2]*q2[2]+q1[3]*q2[3];
}

void rot_axis_to_quater(float angle, const vector<float> axis, vector<float>& quat){
	float theta = angle * M_PI / 180.0f;
	quat[0] = cos(theta / 2.0f);
	for(int i = 0; i < 3; i++){
		quat[i + 1] = sin(theta / 2.0f) * axis[i];
	}
}

vector<float> slerp(const float t,const vector<float>& q1,const vector<float>& q2)
{
	vector<float> result(4,0);
	vector<float> q1_copy=q1;
	vector<float> q2_copy=q2;
	if(quaternion_dot_product(q1,q2)<0){
		for(int i=0;i<4;i++){
			q1_copy[i]*=-1.0f;
		}
	}
	float q1_magnitude=sqrt(quaternion_magnitude_squared(q1_copy));
	float q2_magnitude=sqrt(quaternion_magnitude_squared(q2_copy));
	for(int i=0;i<4;i++){
		q1_copy[i]/=q1_magnitude;
		q2_copy[i]/=q2_magnitude;
	}
	if(t<1e-3 || 1-t<1e-3){
		for(int i=0;i<4;i++){
			result[i]=(1.0f-t)*q1_copy[i]+t*q2_copy[i];
		}
		return result;
	}
	float alpha=acos(quaternion_dot_product(q1_copy,q2_copy));
	for(int i=0;i<4;i++){
			result[i]=sin((1.0f-t)*alpha)/sin(alpha)*q1_copy[i]+sin(t*alpha)/sin(alpha)*q2_copy[i];
	}
	return result;
}
 

////function called by avatar thread
void* avatar_thread_update(void* key_input)
{
	bool anim_done = true;
	while(true){
		////wait until receiving the signal to update
		pthread_mutex_lock(&mutex); 

		// Set idle state for animation
		if(anim_done){
			pos_ctl[0] = pos_ctl[3];
			pos_ctl[1] = pos_ctl[3];
			pos_ctl[2] = pos_ctl[3];
			anim_t = 0.0f;
		}

		while (!update_avatar){
			pthread_cond_wait(&avatar_update_cond, &mutex); 
		}

		////update the motion of the corresponding avatar
		vector<float> next_dir(3,0);
		if(avatar_key == 'w'){//move forward
			//translate[0][0]+=0.1f;
			next_dir[0] += 1.0f;
		}
		if(avatar_key == 's'){//move backward
			//translate[0][0]-=0.1f;
			next_dir[0] -= 1.0f;
		}
		if(avatar_key == 'a'){//move left
			//translate[0][2]-=0.1f;
			next_dir[2] -= 1.0f;
		}
		if(avatar_key == 'd'){//move right
			//translate[0][2]+=0.1f;
			next_dir[2] += 1.0f;
		}
		if(avatar_key == 'n')//turn left
			angle[0]+=pi/2.0f;
		if(avatar_key == 'm')//turn right
			angle[0]-=pi/2.0f;
		
		// New command or command interrupting animation
		if(avatar_key != '\0'){
			if(anim_done == false){
				pos_ctl[0][0] = translate[0][0];
				pos_ctl[0][1] = translate[0][1];
				pos_ctl[0][2] = translate[0][2];
				pos_ctl[1] = pos_ctl[0];
				pos_ctl[2] = pos_ctl[0];
			}
			anim_done = false;
			anim_t = 0.0f;
			pos_ctl[3] = vec_add(3, false, pos_ctl[1], next_dir);
		}
		
		vector<float> next_pos = cubic_bezier(anim_t, pos_ctl[0], pos_ctl[1], pos_ctl[2], pos_ctl[3]);
		if(avatar_key != '\0')
			printf("Prev Pos: %f\t%f\t%f\n",  next_pos[0],  next_pos[1],  next_pos[2]);
		for(int i = 0; i < 3; i++){
			translate[0][i] = next_pos[i];
		}

		update_avatar = false; 
		anim_done = (anim_t > .999999999f);
		anim_t += .01f;
		avatar_key = '\0';
		pthread_mutex_unlock(&mutex);
	}
	return 0;
}
////initialize threads
void init_threads()
{
    ////initialize mutex
	pthread_mutex_init(&mutex,0);

    ////initialize data
	angle[0]=0;angle[1]=90.0f;
	angular_velocity[0]=180.0f;angular_velocity[1]=270.0f;

	////create threads
	pthread_t threads[thread_num];
	////create threads, return 0 if the thread is correctly created
	int rc = pthread_create(&threads[0], NULL, avatar_thread_update, (void*)avatar_key);   
	if(rc){std::cerr<<"cannot create thread "<<0<<std::endl;}
}
////destroy threads
void destroy_threads()
{
    pthread_mutex_destroy(&mutex);
	pthread_cond_destroy(&avatar_update_cond);
}
////signal the avatar thread
void update()
{
	pthread_mutex_lock(&mutex);
	update_avatar=true;
	pthread_cond_signal(&avatar_update_cond);
	pthread_mutex_unlock(&mutex);

}

////opengl functions
void init()
{
	float ambient_color[]={0.5f,0.5f,0.5f,1.0f};
	float light_pos[]={1.0f,1.0f,1.5f,0};
	float light_color[]={1.0f,1.0f,1.0f,1.0f};
	
	for(int i = 0; i < 4; i++)
		pos_ctl.push_back(vector<float>(3, 0));

	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_color);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient_color);
	
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void display()
{
	pthread_mutex_lock(&mutex);

	glClearColor(0,0,0,1);
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(-1.0f,3.0f,0,1.0f,0,0,0,1.0f,0);

    ////draw a cube controlled by the avatar thread
	glPushMatrix();
	glTranslatef(translate[0][0],translate[0][1],translate[0][2]);
	glRotatef(angle[0],axis[0][0],axis[0][1],axis[0][2]);
	glEnable(GL_COLOR_MATERIAL);
	glColor3fv(color[0]);
	glutSolidCube(0.5f);
	glDisable(GL_COLOR_MATERIAL);
	glPopMatrix();

	glutSwapBuffers();

	pthread_mutex_unlock(&mutex);
}

void reshape(int w,int h)
{
	window_width=w;window_height=h;
	glViewport(0,0,(GLsizei)window_width,(GLsizei)window_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    gluPerspective(45.0,(GLdouble)window_width/(GLdouble)window_height,0.1,1000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void keyPressed(unsigned char key,int x,int y)
{
	pthread_mutex_lock(&mutex);
	avatar_key=key;
	pthread_mutex_unlock(&mutex);
	update();
	glutPostRedisplay();
}


void timer(int value)
{
	glutPostRedisplay();
	pthread_mutex_lock(&mutex);
	avatar_key = '\0'; // Prevent repeat keypresses
	pthread_mutex_unlock(&mutex);
	update();
	glutTimerFunc(1000/60, timer, window_id);
}

int main(int argc,char* argv[])
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_ALPHA);
	glutInitWindowSize(window_width, window_height);
	window_id = glutCreateWindow("CS248 HW2");
	glutSetWindow(window_id);

	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyPressed);
	glutTimerFunc(1000/60, timer, window_id);

	//debugging code for testing quaternions and slerp
	vector<float> q1(4, 0);
	vector<float> j(3, 0);
	j[1] = 1.0f;
	rot_axis_to_quater(30.0f, j, q1);
	//q1[0]=1.5;q1[1]=2.3;q1[2]=3.7;q1[3]=-1.2;
	vector<float> q2(4,0); q2[0]=1.78; q2[1]=-2.15; q2[2]=-3.1; q2[3]=-1.36;
	vector<float> v(3,0); v[0]=1; v[1]=0; v[2]=0;
	//v[0]=-1.2;v[1]=8.9;v[2]=-100.6;
	float t=.23;
	pthread_mutex_lock(&mutex);
	std::cout<<"magnitude squared "<<quaternion_magnitude_squared(q1)<<std::endl;
	std::cout<<"magnitude inverse "<<quaternion_inverse(q2)[0]<<" "<<quaternion_inverse(q2)[1]<<" "<<quaternion_inverse(q2)[2]<<" "<<quaternion_inverse(q2)[3]<<std::endl;
	std::cout<<"magnitude product "<<quaternion_product(q1,q2)[0]<<" "<<quaternion_product(q1,q2)[1]<<" "<<quaternion_product(q1,q2)[2]<<" "<<quaternion_product(q1,q2)[3]<<std::endl;
	std::cout<<"slerp "<<t<<" "<<slerp(t,q1,q2)[0]<<" "<<slerp(t,q1,q2)[1]<<" "<<slerp(t,q1,q2)[2]<<" "<<slerp(t,q1,q2)[3]<<std::endl;
	std::cout<<"rotated vector "<<rotate_vector(v,q1)[0]<<" "<<rotate_vector(v,q1)[1]<<" "<<rotate_vector(v,q1)[2]<<std::endl;
	pthread_mutex_unlock(&mutex);

	init();
	init_threads();
	glutMainLoop();

    destroy_threads();
	return 0;
}
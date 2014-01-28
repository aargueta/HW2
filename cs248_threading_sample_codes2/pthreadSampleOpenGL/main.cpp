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
vector<float> axis;
float translate[thread_num][3]={0.0f,0,0};
float color[thread_num][3]={1.0f,0,0};
float angle[thread_num]={0};
float angular_velocity[thread_num]={0};
bool update_avatar=false;
unsigned char avatar_key;

float anim_dist = .1f;
float anim_rot = 2.0f;
float anim_speed = .2f;
float anim_t = 0.0f;
vector<vector<float>> pos_ctl;
vector<vector<float>> rot_ctl;
vector<float> curr_quat(4,0);
vector<float> final_quat(4,0);

vector<float> vec_add(int dim, bool sub, vector<float> a, vector<float> b){
	vector<float> c(dim, 0);
	for(int i = 0; i < dim; i++){
		c[i] = a[i] + (sub? -1.0f * b[i] : b[i]);
	}
	return c;
}

float vec_dot_product(int dim, const vector<float> a, const vector<float> b){
	float dot = 0.0f;
	for(int i = 0; i < dim; i++){
		dot += a[i] * b[i];
	}
	return dot;
}

vector<float> vec_scalar_product(int dim, float c, const vector<float> a){
	vector<float> scaled(dim, 0);
	for(int i = 0; i < dim; i++){
		scaled[i] = c * a[i];
	}
	return scaled;
}

vector<float> vec_normalize(int dim, vector<float> a){
	float mag = 0.0f;
	for(int i = 0; i < dim; i++){
		mag += a[i] * a[i];
	}
	mag = sqrt(mag);
	vector<float> normalized(dim, 0);
	if(mag > 0.0f){
		for(int i = 0; i < dim; i++){
			normalized[i] = a[i] / mag;
		}
	}
	return normalized;
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

vector<float> rot_axis_to_quater(float angle, const vector<float> axis){
	vector<float> quat(4,0);
	float theta = angle * M_PI / 180.0f;
	quat[0] = cos(theta / 2.0f);
	for(int i = 0; i < 3; i++){
		quat[i + 1] = sin(theta / 2.0f) * axis[i];
	}
	return quat;
}

vector<float> quater_to_rot_axis(vector<float> quat){
	vector<float> rot_axis(4, 0);
	rot_axis[0] = acos(quat[0]) * M_2_PI * 180.0f;
	float sin_theta2 = sin(acos(quat[0]));
	for(int i = 1; i < 4; i++)
		rot_axis[i] = quat[i] / sin_theta2;
	return rot_axis;
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
		if(alpha < .0000001f)
			result[i]=(1.0f-t)*q1_copy[i]+t*q2_copy[i];
		else
			result[i]=sin((1.0f-t)*alpha)/sin(alpha)*q1_copy[i]+sin(t*alpha)/sin(alpha)*q2_copy[i];
	}
	return result;
}


////function called by avatar thread
void* avatar_thread_update(void* key_input)
{
	bool anim_done = true;
	int rotate_dir = 0;
	while(true){
		//wait until receiving the signal to update
		pthread_mutex_lock(&mutex); 

		// Setup position control curve at the end/beginning of an animation
		if(anim_done){
			if(pos_ctl.size() > 4){
				// We have queued actions, update ctl vector by popping
				pos_ctl.erase(pos_ctl.begin()); // Remove start point
				pos_ctl.erase(pos_ctl.begin()); // Remove starting deriv
				pos_ctl.erase(pos_ctl.begin()); // Remove old ending deriv
				anim_t = 0.0f;
			}else{
				// We're actually done. Update old ending deriv to the end position
				pos_ctl[2] = pos_ctl[3];
			}
		}

		while (!update_avatar){
			pthread_cond_wait(&avatar_update_cond, &mutex); 
		}

		////update the motion of the corresponding avatar
		vector<float> next_dir(3,0);
		if(avatar_key == 'w'){//move forward
			next_dir[0] += anim_dist;
		}
		if(avatar_key == 's'){//move backward
			next_dir[0] -= anim_dist;
		}
		if(avatar_key == 'a'){//move left
			next_dir[2] -= anim_dist;
		}
		if(avatar_key == 'd'){//move right
			next_dir[2] += anim_dist;
		}
		if(avatar_key == 'z'){//move up
			next_dir[1] += anim_dist;
		}
		if(avatar_key == 'x'){//move down
			next_dir[1] -= anim_dist;
		}
		if(avatar_key == 'n'){//bank left
			rot_ctl[0][0] += anim_rot;
			rotate_dir = 0;
		}
		if(avatar_key == 'm'){//bank right
			rot_ctl[0][0] -= anim_rot;
			rotate_dir = 0;
		}
		if(avatar_key == 'j'){//turn left
			rot_ctl[1][0] += anim_rot;
			rotate_dir = 1;
		}
		if(avatar_key == 'k'){//turn right
			rot_ctl[1][0] -= anim_rot;
			rotate_dir = 1;
		}
		if(avatar_key == 'i'){//pitch up
			rot_ctl[2][0] += anim_rot;
			rotate_dir = 2;
		}
		if(avatar_key == 'o'){//pitch down
			rot_ctl[2][0] -= anim_rot;
			rotate_dir = 2;
		}


		// New command or command interrupting animation
		if(avatar_key != '\0'){
			vector<float> scnd_end = pos_ctl[pos_ctl.size() - 2];
			vector<float> end = pos_ctl.back();

			// Calculate default C1 & C2
			vector<float> new_end = vec_add(3, false, end, next_dir);

			float dx = abs(end[0] - new_end[0]);
			float dy = abs(end[1] - new_end[1]);
			float dz = abs(end[2] - new_end[2]);

			vector<float> ctl1(end), ctl2(new_end);
			if(dx > dy && dx > dz){
				// dX is greatest
				ctl1[0] = (end[0] + new_end[0]) / 2.0f;
				ctl2[0] = ctl1[0];
			} else if( dy > dz){
				// dY is greatest
				ctl1[1] = (end[1] + new_end[1]) / 2.0f;
				ctl2[1] = ctl1[1];
			} else{
				// dZ is greatest
				ctl1[2] = (end[2] + new_end[2]) / 2.0f;
				ctl2[2] = ctl1[2];
			}

			// Calculate derivatives for C1 continuity
			vector<float> end_deriv = vec_normalize(3, vec_add(3, true, end, scnd_end));
			ctl1 = vec_add(3, false, end, vec_scalar_product(3, vec_dot_product(3, vec_add(3, true, ctl1, end), end_deriv), end_deriv));

			pos_ctl.push_back(ctl1);
			pos_ctl.push_back(ctl2);
			pos_ctl.push_back(new_end);
			
			// Calculate rotation end point
			if(anim_t > .999999999f)
				curr_quat = final_quat; //rot_axis_to_quater(angle[0], axis);

			//for(int i = 0; i < 3; i++)
			//final_quat = quaternion_product(rot_axis_to_quater(rot_ctl[i][0], vector<float>(rot_ctl[i].begin() + 1, rot_ctl[i].end())), final_quat);
			final_quat = rot_axis_to_quater(rot_ctl[rotate_dir][0], vector<float>(rot_ctl[rotate_dir].begin() + 1, rot_ctl[rotate_dir].end()));
			anim_done = false;
		}
		
		update_avatar = false; 
		anim_done = (anim_t > .999999999f);
		
		if(!anim_done){
			// Calculate interpolated values
			vector<float> next_pos = cubic_bezier(anim_t, pos_ctl[0], pos_ctl[1], pos_ctl[2], pos_ctl[3]);
			vector<float> curr_rot(4, 0);
			curr_rot[0] = angle[0];
			curr_rot[1] = axis[0];
			curr_rot[2] = axis[1];
			curr_rot[3] = axis[2];
			vector<float> temp_quat = slerp(anim_t, curr_quat, final_quat);
			vector<float> next_rot_axis = quater_to_rot_axis(temp_quat);

			// Copy values to OpenGL inputs
			angle[0] = next_rot_axis[0];
			printf("%f - %f*\n", anim_t, angle[0]);
			for(int i = 0; i < 3; i++){
				translate[0][i] = next_pos[i];
				axis[i] = next_rot_axis[i + 1];
			}

			anim_t += anim_speed;
		}
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

	for(int i = 0; i < 4; i++){
		pos_ctl.push_back(vector<float>(3, 0));
		rot_ctl.push_back(vector<float>(4, 0));
	}
	rot_ctl[0][1] = 1.0f;
	rot_ctl[1][2] = 1.0f;
	rot_ctl[2][3] = 1.0f;

	axis = vector<float>(3,0);
	axis[1] = 1.0f;

	//curr_quat[2] = 1.0f;
	curr_quat[0] = 1.0f;
	final_quat[0] = 1.0f;
	//final_quat[2] = 1.0f;

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
	glRotatef(angle[0],axis[0],axis[1],axis[2]);

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
	
	if(key == '+'){
		anim_dist += .1f;
		anim_rot += 1.0f;
		return;
	}else if(key == '-'){
		anim_dist -= .1f;
		anim_rot -= 1.0f;
		return;
	}else if(key == '*'){
		anim_speed *= 2.0f;
		return;
	}else if(key == '/'){
		anim_speed /= 2.0f;
		return;
	}
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
	q1 = rot_axis_to_quater(30.0f, j);
	vector<float> ra1 = quater_to_rot_axis(q1);
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
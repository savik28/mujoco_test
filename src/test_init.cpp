#include "mujoco.h"
#include "mjdata.h"
#include "mjmodel.h"
#include "stdio.h"
#include <iostream>
#include <fstream>

using namespace std;

char error[1000];
mjModel* m;
mjData* d;

int main(void)
{
   ofstream data_file ("data.txt");
   //data_file.open("data.txt");

   mj_activate("/home/student/mjpro140/bin/mjkey.txt");

   // load model from file and check for errors
   m = mj_loadXML("/home/student/mjpro140/model/ur5/ur5.xml", NULL, error, 1000);

   if( !m )
   {
      printf("%s\n", error);
      return 1;
   }
   else
   {
        data_file <<"Model loaded, parsed & converted sucessfully\n"
                  <<endl<<"MODEL_PARAMETERS"<< endl
                  <<"Gen.Coordinates :"<< m->nq <<endl
                  <<"DOF's :"<< m->nv <<endl
                  <<"Bodys :"<< m->nbody <<endl
                  <<"Joints:"<< m->njnt <<endl;

        cout<<endl<<"MODEL_PARAMETERS"<< endl
                 <<"Gen.Coordinates :"<< m->nq <<endl
                 <<"DOF's :"<< m->nv <<endl
                 <<"Bodys :"<< m->nbody <<endl
                 <<"Joints:"<< m->njnt <<endl;
                 //<< m->name_bodyadr[1] << endl;
                            // << *(m->body_mass) <<endl;
    }

   // int=int; float=float; mjNum=double; mjtByte=unsigned char


   cout << m->nnames<<endl;
   int color_size = 4;
   for (int mat_id = 0; mat_id < m->nmat; mat_id++)
   {
       cout <<m->names[m->name_matadr[mat_id]]<<"'s rgba: ";
       for (int color_id = 0; color_id < color_size; color_id++)
       cout<< m->mat_rgba[mat_id*color_size + color_id] << " ";
       cout<<endl;
   }
   cout<<endl;


   // make data corresponding to model
   d = mj_makeData(m);
  //d->xfrc_applied[6] = 1.002877;

   cout<<endl<<endl<<"DATA PARAMETERS:"<<endl
            <<"Sim. Time: "<< d->time << endl
            <<"Pot. Energy: "<< d->energy[1] << endl
            <<"Kin. Energy: "<< d->energy[2] << endl
            <<"Size of Data(d): "<<sizeof(*d) <<"Bytes"<<endl;




   //mjtNum **b = &d->xfrc_applied;


   const char *nm = "model.txt",
              *nam = "data_b4_simu.txt",
              *nma = "data_aftr_simu.txt";

   mj_printModel(m,nm);
   mj_printData(m, d, nam);

   // run simulation for 10 seconds
   int steps=0;
   while( d->time<10)
   {
      mj_step(m, d);
      /*
      cout <<endl<<"STEP:"<< steps <<endl<<"Force_Sensor: ";
      for (int j = 0; j < 3; j++)
      {
        cout<< d->sensordata[0*3 + j] << "  ";
      }
      cout<<endl;

      for (int i=1; i < m->nbody; i++)
      {
          cout<<"BODY_ID: "<<i<<endl<<"Position:";
          for (int j = 0; j < 3; j++)
          {
            cout<< d->xpos[i*3 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cacc: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cacc[i*6 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cfrc_int: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cfrc_int[i*6 + j] << "  ";
          }
          cout<<endl;

          cout<<"Cfrc_ext: ";
          for (int j = 0; j < 6; j++)
          {
            cout<< d->cfrc_ext[i*6 + j] << "  ";
          }
          cout<<endl;
      }*/
      steps++;
   }

   mj_printData(m, d, nma);


   // free model and data, deactivate
   mj_deleteData(d);
   mj_deleteModel(m);
   mj_deactivate();
   data_file.close();

   return 0;
}


/*float **a = &m->geom_rgba;
//int **a = &m->body_parentid;
for (int i = 0; i < 1; i++)
{
    for (int j = 0; j < 8; j++)
    {
       // cout<< a[i][j]<< "  ";
       cout<< (&m->geom_rgba)[i][j]<< "  ";
    }
    cout<<endl;
}
   mjtNum **f = &d->qfrc_applied;
   cout<<"Ext. Force: "<<endl;
   for (int j = 0; j < 6; j++)
         cout<< f[0][j]<< "  ";
   cout<<endl;
   for (int j = 6; j < 12; j++)
         cout<< f[0][j]<< "  ";
    cout<<endl;
   mjtNum **p = &d->xpos;
   for (int i = 0; i < 1; i++)
   {
       cout<<"Body Position: "<<endl;
       for (int j = 0; j < (m->nbody * 3); j++)
       {
         cout<< p[i][j]<< "  ";
       }
       cout<<endl;
   }

   int color_size = 4;
   for (int body_id = 0; body_id < m->nbody; body_id++)
   {
       cout <<"Body_"<<body_id<<" rgba: ";
       for (int color_id = 0; color_id < color_size; color_id++)
       cout<< m->geom_rgba[body_id*color_size + color_id] << " ";
       cout<<endl;
   }
   cout<<endl;



*/

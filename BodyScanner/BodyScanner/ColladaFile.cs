using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Collada141;
using System.ComponentModel;

namespace ColladaUsage
{
    class ColladaFile
    {
        static public void CreateFile(BackgroundWorker worker)
        {
            //createModel();
            // Load the Collada model
            double[] data;
            System.IO.StreamReader bodyData = new System.IO.StreamReader("data_body.txt");
            string verticesString = bodyData.ReadLine();
            int vertices=int.Parse(verticesString.Substring(verticesString.IndexOf('(')+1,verticesString.Length-verticesString.IndexOf('(')-2));
            string tempLine;
            data = new double[vertices*3];
            int row = 0;
            while (!(tempLine = bodyData.ReadLine()).Contains("NORMALES"))
            {
                data[row] = double.Parse(tempLine.Substring(0, tempLine.IndexOf(',')));
                tempLine = tempLine.Substring(tempLine.IndexOf(',') + 2);
                row++;

                data[row] = double.Parse(tempLine.Substring(0, tempLine.IndexOf(',')));
                tempLine = tempLine.Substring(tempLine.IndexOf(',') + 2);
                row++;

                data[row] = double.Parse(tempLine.Substring(0, tempLine.IndexOf(',')));
                tempLine = tempLine.Substring(tempLine.IndexOf(',') + 2);
                row++;

                //worker.ReportProgress(75 + row * 10/(vertices * 3));
            }

            while (!(tempLine = bodyData.ReadLine()).Contains("TRIANGULOS"))
            {
                
            }
            //worker.ReportProgress(90);

            int triangles = int.Parse(tempLine.Substring(tempLine.IndexOf('(') + 1, tempLine.Length - tempLine.IndexOf('(') - 2));
            string trianglesIndex = bodyData.ReadLine();

            bodyData.Close();
           
            COLLADA model = COLLADA.Load("default.dae");
            (((model.Items[2] as library_geometries).geometry[0].Item as mesh).source[0].Item as float_array).count = (ulong)data.Length;
            (((model.Items[2] as library_geometries).geometry[0].Item as mesh).source[0].Item as float_array).Values = new double[data.Length];

            for (int i = 0; i < data.Length;i++ )
                (((model.Items[2] as library_geometries).geometry[0].Item as mesh).source[0].Item as float_array).Values[i] = data[i];

            ((model.Items[2] as library_geometries).geometry[0].Item as mesh).source[0].technique_common.accessor.count = (ulong)(data.Length/3);

            (((model.Items[2] as library_geometries).geometry[0].Item as mesh).Items[0] as triangles).count=(ulong)triangles;
            (((model.Items[2] as library_geometries).geometry[0].Item as mesh).Items[0] as triangles).p = trianglesIndex;
           
             //worker.ReportProgress(95);
 
            // Save the model
            model.Save("model_body.dae");

            //worker.ReportProgress(100);
        }

        static public double distance3D(double x1, double y1, double z1, double x2, double y2, double z2)
        {
            //     __________________________________
            //d = &#8730; (x2-x1)^2 + (y2-y1)^2 + (z2-z1)^2
            //

            //Our end result
            double result = 0;
            //Take x2-x1, then square it
            double part1 = Math.Pow((x2 - x1), 2);
            //Take y2-y1, then sqaure it
            double part2 = Math.Pow((y2 - y1), 2);
            //Take z2-z1, then square it
            double part3 = Math.Pow((z2 - z1), 2);
            //Add both of the parts together
            double underRadical = part1 + part2 + part3;
            //Get the square root of the parts
            result = Math.Sqrt(underRadical);
            //Return our result
            return result;
        }
    }
}

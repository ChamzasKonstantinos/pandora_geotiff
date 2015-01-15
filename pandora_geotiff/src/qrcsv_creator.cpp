std::string getDateAndTime();
std::string getQrTime(time_t qrTime);
void generateQrCsv(std::string missionName);






std::string ObjectsWriter::getDateAndTime(){

  time_t t = time(0); // get time now
  struct tm * now = localtime( & t );
  char buf[20];
  std::stringstream ss;
  
  strftime(buf, sizeof(buf), "%F", now);
  
  ss << buf << "; ";
  
  strftime(buf, sizeof(buf), "%T", now);
  
  ss << buf;
  
  std::string str = ss.str();

  return str;

}

std::string ObjectsWriter::getQrTime(time_t qrTime){
  
  struct tm * now = localtime( & qrTime );
  char buf[10];
  std::stringstream ss;
  strftime(buf, sizeof(buf), "%T", now);
  ss << buf;
  
  std::string str = ss.str();

  return str;
}

//~ void GeotiffCreator::generateQrCsv(){
//~ 
//~ QString filenameString("/RC_2013_PANDORA_");
//~ filenameString.append(missionName);
//~ filenameString.append("_qr.csv");
//~ 
//~ QString filepath = homeFolderString;
//~ filepath = filepath.append("/Desktop/");
//~ filepath.append(filenameString);
//~ std::string filepathStr = filepath.toUtf8().constData();
//~ 
//~ std::cout << filepathStr << "\n";
//~ 
//~ std::ofstream csvFile;
//~ csvFile.open (filepathStr.c_str());
//~ 
//~ // ---csvFile---
//~ // Resko Koblenz, Germany
//~ // 2013-06-23; 14:37:03
//~ // Semi1
//~ //
//~ // 1;14:28:01;Y_1_1_chair_yoked;-8.29994;-2.29014

//~ csvFile << "PANDORA AUTh, Greece" << std::endl;
//~ csvFile << getDateAndTime() << std::endl;
//~ csvFile << missionName.toUtf8().constData() << std::endl << std::endl;
//~ std::string qrWorldTime[qrSize];
//~ 
//~ for(int i=0; i<qrSize; i++){
//~ qrWorldTime[i] = getQrTime(qrTime[i]);
//~ }
//~ 
//~ for(int i=0; i<qrSize; i++)
//~ csvFile << i+1 << ";" << qrWorldTime[i] << ";" << qrType[i] << ";" << qrWorldX[i] << ";" << qrWorldY[i] << std::endl;
//~ 
//~ 
//~ csvFile.close();
//~ 
//~ }

void ObjectsWriter::generateQrCsv(std::string missionName){

  std::string filenameString("/RC_2015_PANDORA_");
  filenameString.append(missionName);
  filenameString.append("_qr.csv");

  //~ passwd* pw = getpwuid(getuid());
  //~ std::string filepath(pw->pw_dir);
  //~ filepath = filepath.append("/Desktop/");
  //~ filepath.append(filenameString);
//~ 
  //~ std::ofstream csvFile;
  //~ csvFile.open (filepath.c_str());
//~ 
  //~ csvFile << "PANDORA AUTh, Greece" << std::endl;
  //~ csvFile << getDateAndTime() << std::endl;
  //~ csvFile.close();
  //~ for (int i = 0 ; i< qrs_.size(); i++){
      //~ csvFile << i+1 << ";" << qrs_[i].stamp << ";" << qrType[i] << ";" << qrWorldX[i] << ";" << qrWorldY[i] << std::endl;
      //~ float y  = qrs_[i].pose.position.y + origin.y();
      //~ coords = Eigen::Vector2f(x*cos(yaw) + y*sin(yaw),y*cos(yaw) +x*sin(yaw));
      //~ txt = boost::lexical_cast<std::string>(i+1);
     //~ 
     //~ interface->drawObjectOfInterest(coords,QRS_COLOR ,TXT_COLOR, QRS_SHAPE,txt,QRS_SIZE);
    //~ }
}

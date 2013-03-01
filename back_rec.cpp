void testDatabase(const vector<vector<vector<float> > > &features,const vector<vector<vector<float> > > &features2)
{

    float hits = 0;
    float success = 0;
    float hitsrgb = 0;
    float successrgb = 0;

    float combo_query=0;
    float combo_success=0;

    NarfVocabulary voc(filename_voc_3d);
    NarfDatabase db(voc, false);

    Surf128Vocabulary voc2(filename_voc_rgb);
    Surf128Database db2(voc2, false);

    for (int i = 0; i < files_list_3d.size(); i++) {
        db.add(features[i]);
    }

    for (int i = 0; i < files_list_rgb.size(); i++) {
        db2.add(features2[i]);
    }


    //valutazione unificata
    for (int kk = 0; kk<files_list_3d.size(); kk++ ){
        QueryResults ret;
        db.query(features[kk], ret,10);
        QueryResults ret2;
        db2.query(features2[kk], ret2,10);

        cout << "3d" <<endl << ret << endl;
        cout << "rgb" <<endl << ret2 << endl;

        map<int,int> pivot,pivot2;
        cout << "ret: " << ret.size() << endl;
        vector<string> a;
        boost::split(a, registro_aux2.find(kk)->second, boost::is_any_of("=>"));
        a.erase( std::remove_if( a.begin(), a.end(), boost::bind( &std::string::empty, _1 ) ), a.end());
        //inizializzo
        for(int yu = 0; yu < ret.size(); yu++){
            //tolgo immagine self
            vector<string> b;
            boost::split(b, registro_aux2.find(ret[yu].Id)->second, boost::is_any_of("=>"));
            b.erase( std::remove_if( b.begin(), b.end(), boost::bind( &std::string::empty, _1 ) ), b.end());
            if (!boost::iequals(a[0],b[0])) {
                pivot.insert(PivotMappa(ret[yu].Id,ret.size() - yu));
            }
        }
        for(int yu = 0; yu < ret2.size(); yu++){
            if (pivot.find(ret2[yu].Id) == pivot.end()){
                //tolgo immagine self
                vector<string> b;
                boost::split(b, registro_aux2.find(ret2[yu].Id)->second, boost::is_any_of("=>"));
                b.erase( std::remove_if( b.begin(), b.end(), boost::bind( &std::string::empty, _1 ) ), b.end());

                if (!boost::iequals(a[0],b[0])) {
                    pivot.insert(PivotMappa(ret2[yu].Id,ret2.size() - yu));
                }
            }
            else
            {
                pivot[ret2[yu].Id] = pivot[ret2[yu].Id] + (ret2.size() - yu);
            }
        }
        for(map<int,int>::iterator it=pivot.begin(); it != pivot.end(); it++)
        {
            pivot2.insert(PivotMappa(it->second,it->first));
        }
        combo_query++;
        for(map<int,int>::reverse_iterator it=pivot2.rbegin(); it!=pivot2.rend(); ++it)
        {
            vector<string> a,b;
            boost::split(a, registro_aux2.find(kk)->second, boost::is_any_of("=>"));
            a.erase( std::remove_if( a.begin(), a.end(), boost::bind( &std::string::empty, _1 ) ), a.end());
            boost::split(b, registro_aux2.find(it->second)->second, boost::is_any_of("=>"));
            b.erase( std::remove_if( b.begin(), b.end(), boost::bind( &std::string::empty, _1 ) ), b.end());
            if (!boost::iequals(a[0],b[0])) {
                if (boost::iequals(a[1],b[1])){
                    combo_success++;
                    cout << "OK - Cercata: " << a[0] << " Trovata: " << b[0] << "Entry: " << it->second << " Voto: "<< it->first << endl;
                }
                else
                {
                    cout << "NO - Cercata: " << a[0] << " Trovata: " << b[0] << "Entry: " << it->second << " Voto: "<< it->first << endl;
                }
            }
            break;
        }
    }

    //risultati da considerare
    const int maxx = 2;
    for (int i = 0; i < files_list_3d.size(); i++) {
        QueryResults ret;
        db.query(features[i],ret,maxx);
        cout << "Cerca immagine depth " << i << ". " << " Etichetta: "<< registro_aux.find(i)->second << endl;

        vector<string> a;
        boost::split(a, registro_aux2.find(i)->second, boost::is_any_of("=>"));
        a.erase( std::remove_if( a.begin(), a.end(), boost::bind( &std::string::empty, _1 ) ), a.end());
        hits = hits+1;
        vector<string> b;
        bool myself = false;
        for (int yy = 0; yy < ret.size() ; yy++) {
            boost::split(b, registro_aux2.find(ret[yy].Id)->second, boost::is_any_of("=>"));
            b.erase( std::remove_if( b.begin(), b.end(), boost::bind( &std::string::empty, _1 ) ), b.end());
            if (!boost::iequals(a[0],b[0])) {
                StringFunctions::trim(a[1]);
                StringFunctions::trim(b[1]);
                if (strcmp(a[1].c_str(),b[1].c_str())==0) {
                    if (yy == (ret.size() -1)) {
                        if (myself) {
                            cout << "OK - ID: " <<ret[yy].Id << ", " << "Score:" << ret[yy].Score << ", Etichetta: " << registro_aux2.find(ret[yy].Id)->second << endl;
                            success = success+1;
                            myself = false;
                            break;
                        } else {
                            cout << " FALSO OK - ID: " <<ret[yy].Id << ", " << "Score:" << ret[yy].Score << ", Etichetta: " << registro_aux2.find(ret[yy].Id)->second << endl;
                        }
                    } else {
                        cout << "OK - ID: " <<ret[yy].Id << ", " << "Score:" << ret[yy].Score << ", Etichetta: " << registro_aux2.find(ret[yy].Id)->second << endl;
                        success = success+1;
                        myself = false;
                        break;
                    }
                } else {
                    cout << "NO - ID: " <<ret[yy].Id << ", " << "Score:" << ret[yy].Score << ", Etichetta: " << registro_aux2.find(ret[yy].Id)->second << endl;
                    if (yy == (ret.size() - 1)) {
                        myself = false;
                        break;
                    }
                }
            } else {
                myself = true;
                cout << "SELF - ID: " <<ret[yy].Id << ", " << "Score:" << ret[yy].Score << ", Etichetta: " << registro_aux2.find(ret[yy].Id)->second << endl;
            }
        }
        cout <<endl;
    }
    cout << "QUERY 3D TERMINATE." << endl<<endl;
    for (int i = 0; i < files_list_rgb.size(); i++) {
        QueryResults ret2;
        db2.query(features2[i], ret2,maxx);
        cout << "Cerca immagine rgb" << i << ". " << " Etichetta: "<< registro_aux_rgb.find(i)->second << endl;
        vector<string> a;
        boost::split(a, registro_aux3.find(i)->second, boost::is_any_of("=>"));
        a.erase( std::remove_if( a.begin(), a.end(), boost::bind( &std::string::empty, _1 ) ), a.end());
        hitsrgb = hitsrgb+1;
        vector<string> b;
        bool myself = false;
        for (int yy = 0; yy < ret2.size(); yy++) {
            boost::split(b, registro_aux3.find(ret2[yy].Id)->second, boost::is_any_of("=>"));
            b.erase( std::remove_if( b.begin(), b.end(), boost::bind( &std::string::empty, _1 ) ), b.end());
            if (!boost::iequals(a[0],b[0])) {
                StringFunctions::trim(a[1]);
                StringFunctions::trim(b[1]);
                if (strcmp(a[1].c_str(),b[1].c_str())==0) {
                    if (yy == (ret2.size() -1)) {
                        if (myself) {
                            cout << "OK - ID: " <<ret2[yy].Id << ", " << "Score:" << ret2[yy].Score << ", Etichetta: " << registro_aux3.find(ret2[yy].Id)->second << endl;
                            successrgb = successrgb+1;
                            myself = false;
                            break;
                        } else {
                            cout << " FALSO OK - ID: " <<ret2[yy].Id << ", " << "Score:" << ret2[yy].Score << ", Etichetta: " << registro_aux3.find(ret2[yy].Id)->second << endl;
                        }
                    } else {
                        cout << "OK - ID: " <<ret2[yy].Id << ", " << "Score:" << ret2[yy].Score << ", Etichetta: " << registro_aux3.find(ret2[yy].Id)->second << endl;
                        successrgb = successrgb+1;
                        myself = false;
                        break;
                    }
                } else {
                    cout << "NO - ID: " <<ret2[yy].Id << ", " << "Score:" << ret2[yy].Score << ", Etichetta: " << registro_aux3.find(ret2[yy].Id)->second << endl;
                    if (yy == (ret2.size() -1)) {
                        myself = false;
                        break;
                    }
                }
            } else {
                myself = true;
                cout << "SELF - ID: " <<ret2[yy].Id << ", " << "Score:" << ret2[yy].Score << ", Etichetta: " << registro_aux3.find(ret2[yy].Id)->second << endl;
            }
        }
        cout <<endl;
    }
    cout << "QUERY RGB TERMINATE." << endl;



    cout << endl;
    float prec = (success/hits)*100;
    string ouy = boost::lexical_cast<std::string>(prec);
    string suc = boost::lexical_cast<std::string>(success);
    string hitt = boost::lexical_cast<std::string>(hits);
    cout << "Precisione 3D : " << ouy << " %"<< endl;
    cout <<  " -> "<<suc<< " successi su "<< hitt << " query. "<< endl;

    float precrgb = (successrgb/hitsrgb)*100;
    string ouyrgb = boost::lexical_cast<std::string>(precrgb);
    string sucrgb = boost::lexical_cast<std::string>(successrgb);
    string hittrgb = boost::lexical_cast<std::string>(hitsrgb);
    cout << "Precisione RGB : " << ouyrgb << " %"<< endl;
    cout <<  " -> "<<sucrgb<< " successi su "<< hittrgb << " query. "<< endl;

    float preccombo = (combo_success/combo_query)*100;
    string ouycombo = boost::lexical_cast<std::string>(preccombo);
    string succombo = boost::lexical_cast<std::string>(combo_success);
    string hittcombo = boost::lexical_cast<std::string>(combo_query);
    cout << "Precisione Metodo Combinato : " << ouycombo << " %"<< endl;
    cout <<  " -> "<<succombo<< " successi su "<< hittcombo << " query. "<< endl;

    db.save("db3d.yml.gz");
    db2.save("dbrgb.yml.gz");
}

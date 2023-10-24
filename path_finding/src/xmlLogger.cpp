#include"xmlLogger.h"
using namespace tinyxml2;

bool XmlLogger::createLog(const char *FileName)
{
    if (loglevel == CN_LOGLVL_NO)
        return true;

    std::string value(FileName);
    size_t dotPos = value.find_last_of(".");
    if(dotPos != std::string::npos)
        value.insert(dotPos,CN_LOG);
    else
        value += CN_LOG;
    LogFileName = value;

    std::ofstream out(LogFileName);
    out<<"<?xml version=\"1.0\" ?>\n<root>\n</root>";
    out.close();
    doc = new XMLDocument;
    doc->LoadFile(LogFileName.c_str());
    if(!doc)
        return false;
    XMLElement *root = doc->FirstChildElement(CNS_TAG_ROOT);
    root->LinkEndChild(doc->NewElement(CNS_TAG_LOG));

    return true;
}

void XmlLogger::writeToLogInput(const char *taskName, const char *mapName, const char *configName, const char *obstaclesName)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    else if(loglevel == CN_LOGLVL_NORM)
    {
        XMLElement *log = doc->FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);
        XMLElement *element = doc->NewElement(CNS_TAG_TASKFN);
        element->LinkEndChild(doc->NewText(taskName));
        log->LinkEndChild(element);
        element = doc->NewElement(CNS_TAG_MAPFN);
        element->LinkEndChild(doc->NewText(mapName));
        log->LinkEndChild(element);
        element = doc->NewElement(CNS_TAG_CONFIGFN);
        element->LinkEndChild(doc->NewText(configName));
        log->LinkEndChild(element);
        if(obstaclesName)
        {
            element = doc->NewElement(CNS_TAG_OBSFN);
            element->LinkEndChild(doc->NewText(obstaclesName));
            log->LinkEndChild(element);
        }
    }
    else if(loglevel == CN_LOGLVL_ALL)
    {
        if(taskName == mapName)//i.e. all_in_one
        {
            writeToLogFile(taskName);
        }
        else
        {
            if(obstaclesName)
                writeToLogFile(obstaclesName);
            writeToLogFile(configName);
            writeToLogFile(mapName);
            writeToLogFile(taskName);
        }
    }

}

void XmlLogger::writeToLogFile(const char *fileName)
{
    XMLDocument file;
    file.LoadFile(fileName);
    XMLNode *prev = nullptr;
    for(XMLNode* node = file.RootElement()->FirstChild(); node; node = node->NextSibling())
    {
        XMLNode *clone = node->DeepClone(doc);
        if(!prev)
            doc->RootElement()->InsertFirstChild(clone);
        else
            doc->RootElement()->InsertAfterChild(prev, clone);
        prev = clone;
    }
}

void XmlLogger::saveLog()
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    doc->SaveFile(LogFileName.c_str());
}

void XmlLogger::writeToLogSummary(const SearchResult &sresult)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    element->LinkEndChild(doc->NewElement(CNS_TAG_SUM));
    element = element->FirstChildElement(CNS_TAG_SUM);
    element->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.runtime));
    element->SetAttribute(CNS_TAG_ATTR_TRIES, sresult.tries);
    element->SetAttribute(CNS_TAG_ATTR_AGENTSSOLVED, ((std::to_string(sresult.agentsSolved) + " (" + std::to_string(float(sresult.agentsSolved*100)/sresult.agents)+"%)")).c_str());
    element->SetAttribute(CNS_TAG_ATTR_FLOWTIME, float(sresult.flowtime));
    element->SetAttribute(CNS_TAG_ATTR_MAKESPAN, float(sresult.makespan));
}

bool onSameLine(int x1, int y1, int x2, int y2, int x3, int y3) {
    // Handle horizontal line case
    if (y1 == y2 && y2 == y3) {
        return true;
    }
    // Handle vertical line case
    if (x1 == x2 && x2 == x3) {
        return true;
    }
    // Handle general case
    return (y2 - y1) * (x3 - x2) == (y3 - y2) * (x2 - x1);
}

// 将搜索结果中的路径信息写入到日志中。对于每个代理，它都会记录其起始和结束位置、大小、速度等属性，以及其实际的移动路径。
void XmlLogger::writeToLogPath(const SearchResult &sresult, const Task &task, const Config &config)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    XMLElement *element=doc->FirstChildElement(CNS_TAG_ROOT);
    element = element->FirstChildElement(CNS_TAG_LOG);
    XMLElement *agent_elem, *path;
    for(unsigned int i = 0; i < task.getNumberOfAgents(); i++)
    {
        Agent agent = task.getAgent(i);
        agent_elem = doc->NewElement(CNS_TAG_AGENT);

        // 为代理元素设置几个属性，包括其ID、起始位置的x和y坐标。
        agent_elem->SetAttribute(CNS_TAG_ATTR_ID, agent.id.c_str());
        agent_elem->SetAttribute(CNS_TAG_ATTR_SX, agent.start_j);
        agent_elem->SetAttribute(CNS_TAG_ATTR_SY, agent.start_i);

        // 如果配置中设置了考虑转向，则还要为代理设置起始方向属性。
        if(config.planforturns)
            agent_elem->SetAttribute(CNS_TAG_ATTR_SH, float(agent.start_heading));
        
        // 为代理设置目标位置的x和y坐标属性。
        agent_elem->SetAttribute(CNS_TAG_ATTR_GX, agent.goal_j);
        agent_elem->SetAttribute(CNS_TAG_ATTR_GY, agent.goal_i);

        // 如果配置中设置了考虑转向，则为代理设置目标方向属性。如果方向小于0，设置为预定义的值；否则，设置为实际的方向值。
        if(config.planforturns)
        {
            if(agent.goal_heading < 0)
                agent_elem->SetAttribute(CNS_TAG_ATTR_GH, CNS_HEADING_WHATEVER);
            else
                agent_elem->SetAttribute(CNS_TAG_ATTR_GH, float(agent.goal_heading));
        }
        // 为代理设置其他属性，如大小、移动速度和旋转速度。
        agent_elem->SetAttribute(CNS_TAG_ATTR_SIZE, float(agent.size));
        agent_elem->SetAttribute(CNS_TAG_ATTR_MSPEED, float(agent.mspeed));
        agent_elem->SetAttribute(CNS_TAG_ATTR_RSPEED, float(agent.rspeed));
        element->LinkEndChild(agent_elem); // 将代理元素添加到日志中。
        path = doc->NewElement(CNS_TAG_PATH); // 为代理创建一个新的路径元素。
        
        // 检查是否为该代理找到了路径。根据是否找到路径，为路径元素设置相应的属性，如路径是否找到、运行时间和路径长度。
        if(sresult.pathInfo[i].pathfound)
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_TRUE);
            path->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.pathInfo[i].runtime));
            path->SetAttribute(CNS_TAG_ATTR_DURATION, float(sresult.pathInfo[i].pathlength));
        }
        else
        {
            path->SetAttribute(CNS_TAG_ATTR_PATHFOUND, CNS_TAG_ATTR_FALSE);
            path->SetAttribute(CNS_TAG_ATTR_RUNTIME, float(sresult.pathInfo[i].runtime));
            path->SetAttribute(CNS_TAG_ATTR_DURATION, 0);
        }

        // 将路径元素添加到代理元素中。
        agent_elem->LinkEndChild(path);

        // 如果为代理找到了路径：
        if (sresult.pathInfo[i].pathfound)
        {
            // 初始化两个迭代器（iter 和 it）和部分编号。声明一个部分元素指针。
            auto it = sresult.pathInfo[i].sections.begin();
            auto iter = sresult.pathInfo[i].sections.begin();
            iter++;
            int partnumber(0);
            XMLElement *part;

            if (sresult.pathInfo[i].sections.size() == 2) {
                while(it != --sresult.pathInfo[i].sections.end())
                {
                    // 为该部分创建一个新的XML元素，并设置其ID和起始位置的属性。
                    part = doc->NewElement(CNS_TAG_SECTION);
                    part->SetAttribute(CNS_TAG_ATTR_ID, partnumber);
                    part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
                    part->SetAttribute(CNS_TAG_ATTR_SY, it->i);

                    // 如果配置中设置了考虑转向，则还要为部分设置起始方向属性。
                    if(config.planforturns)
                        part->SetAttribute(CNS_TAG_ATTR_SH, float(it->heading));
                    
                    // 更新迭代器并为部分设置目标位置的属性。
                    part->SetAttribute(CNS_TAG_ATTR_GX, iter->j);
                    part->SetAttribute(CNS_TAG_ATTR_GY, iter->i);

                    // 如果配置中设置了考虑转向，则还要为部分设置目标方向属性。
                    if(config.planforturns)
                        part->SetAttribute(CNS_TAG_ATTR_GH, float(iter->heading));
                    
                    // 持续时间属性。
                    part->SetAttribute(CNS_TAG_ATTR_DURATION, float(iter->g - it->g)); 
                    
                    // 添加到路径元素中。
                    path->LinkEndChild(part);
                    it = iter;
                    iter++;
                    partnumber++;
                }
            }
            else{
                // 再添加一个，用来判断是不是在一条线上，合并轨迹
                auto nextIter = sresult.pathInfo[i].sections.begin();
                nextIter++;
                nextIter++;

                while(it != --sresult.pathInfo[i].sections.end())
                {
                    // 为该部分创建一个新的XML元素，并设置其ID和起始位置的属性。
                    part = doc->NewElement(CNS_TAG_SECTION);
                    part->SetAttribute(CNS_TAG_ATTR_ID, partnumber);
                    part->SetAttribute(CNS_TAG_ATTR_SX, it->j);
                    part->SetAttribute(CNS_TAG_ATTR_SY, it->i);

                    // 如果配置中设置了考虑转向，则还要为部分设置起始方向属性。
                    if(config.planforturns)
                        part->SetAttribute(CNS_TAG_ATTR_SH, float(it->heading));
                    
                    while (nextIter != sresult.pathInfo[i].sections.end() && 
                            onSameLine(it->j, it->i, iter->j, iter->i, nextIter->j, nextIter->i)){
                        iter = nextIter;
                        ++nextIter;
                    }

                    // 更新迭代器并为部分设置目标位置的属性。
                    part->SetAttribute(CNS_TAG_ATTR_GX, iter->j);
                    part->SetAttribute(CNS_TAG_ATTR_GY, iter->i);

                    // 如果配置中设置了考虑转向，则还要为部分设置目标方向属性。
                    if(config.planforturns)
                        part->SetAttribute(CNS_TAG_ATTR_GH, float(iter->heading));
                    
                    // 持续时间属性。
                    part->SetAttribute(CNS_TAG_ATTR_DURATION, float(iter->g - it->g)); 
                    
                    // 添加到路径元素中。
                    path->LinkEndChild(part);

                    it = iter;
                    iter = nextIter;
                    partnumber++;
                }
            }
        }
    }
}

// 此函数将地图数据和搜索结果中的路径信息写入到日志中。
// 地图被表示为一个二维数组，其中每个元素是一个整数。此函数会将路径上的每个位置标记为“*”，并将结果写入到日志中。
void XmlLogger::writeToLogMap(const Map &map, const SearchResult &sresult)
{
    if (loglevel == CN_LOGLVL_NO)
        return;
    std::string text;
    std::vector<int> curLine(map.width, 0);
    XMLElement *element = doc->FirstChildElement(CNS_TAG_ROOT)->FirstChildElement(CNS_TAG_LOG);
    element->LinkEndChild(doc->NewElement(CNS_TAG_PATH));
    element = element->FirstChildElement(CNS_TAG_PATH);
    XMLElement *msg;

    for(int i = 0; i < map.height; i++)
    {
        msg = doc->NewElement(CNS_TAG_ROW);
        msg->SetAttribute(CNS_TAG_ATTR_NUM, i);
        text.clear();
        std::list<Node>::const_iterator iter;
        for(unsigned int k = 0; k < sresult.agents; k++)
            for(iter = sresult.pathInfo[k].path.begin(); iter != sresult.pathInfo[k].path.end(); iter++)
                if((*iter).i == i)
                    curLine[(*iter).j] = 1;

        for(int j = 0; j < map.width; j++)
            if(curLine[j] != 1)
                text += std::to_string(map.Grid[i][j]) + " ";
            else
            {
                text += "* ";
                curLine[j] = 0;
            }
        msg->LinkEndChild(doc->NewText(text.c_str()));
        element->LinkEndChild(msg);
    }
}

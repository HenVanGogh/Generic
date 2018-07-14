#pragma once
#include "Commun.h"
#include "VirtualModel.h"




class zweiAxedCommun : private Commun
{

public:
	zweiAxedCommun(char TAG);

	void proceed(char t_tag);
	

};

class DreiAxedCommun : private Commun
{
public:
	DreiAxedCommun(char TAG);

	void proceed(char t_tag);

};


class DemoCommun : private Commun
{
public:
	DemoCommun(char TAG);

	void proceed(char t_tag);

};

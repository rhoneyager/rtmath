#include "stdafx.h"
#include "glgraph.h"

#include <QtCore/QtPlugin>
#include "glgraphplugin.h"


glgraphPlugin::glgraphPlugin(QObject *parent)
	: QObject(parent)
{
	initialized = false;
}

void glgraphPlugin::initialize(QDesignerFormEditorInterface */*core*/)
{
	if (initialized)
		return;

	initialized = true;
}

bool glgraphPlugin::isInitialized() const
{
	return initialized;
}

QWidget *glgraphPlugin::createWidget(QWidget *parent)
{
	return new glgraph(parent);
}

QString glgraphPlugin::name() const
{
	return "glgraph";
}

QString glgraphPlugin::group() const
{
	return "My Plugins";
}

QIcon glgraphPlugin::icon() const
{
	return QIcon();
}

QString glgraphPlugin::toolTip() const
{
	return QString();
}

QString glgraphPlugin::whatsThis() const
{
	return QString();
}

bool glgraphPlugin::isContainer() const
{
	return false;
}

QString glgraphPlugin::domXml() const
{
	return "<widget class=\"glgraph\" name=\"glgraph\">\n"
		" <property name=\"geometry\">\n"
		"  <rect>\n"
		"   <x>0</x>\n"
		"   <y>0</y>\n"
		"   <width>100</width>\n"
		"   <height>100</height>\n"
		"  </rect>\n"
		" </property>\n"
		"</widget>\n";
}

QString glgraphPlugin::includeFile() const
{
	return "glgraph.h";
}

Q_EXPORT_PLUGIN2(glgraph, glgraphPlugin)

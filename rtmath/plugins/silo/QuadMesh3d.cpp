#include <iostream>
#include <stdio.h>
#include "QuadMesh3d.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>

namespace rtmath {
	namespace plugins {
		namespace silo {
			
					QuadMesh3D::VectorData::VectorData(const std::string &n, int nx, int ny, int nz, bool node) : name(n)
					{
						xdim = nx;
						ydim = ny;
						zdim = nz;
						int ndata;
						if (node)
							ndata = xdim * ydim * zdim;
						else
							ndata = (xdim - 1) * (ydim - 1) * (zdim - 1);
						xd = new float[ndata];
						yd = new float[ndata];
						zd = new float[ndata];
						nodal = node;
					}

					QuadMesh3D::VectorData::~VectorData()
					{
						delete[] xd;
						delete[] yd;
						delete[] zd;
					}

					void QuadMesh3D::VectorData::SetZonalValue(int x, int y, int z, float val[3])
					{
						int index = z*((ydim - 1)*(xdim - 1)) + y*(xdim - 1) + x;
						xd[index] = val[0];
						yd[index] = val[1];
						zd[index] = val[2];
					}

					void QuadMesh3D::VectorData::SetNodalValue(int x, int y, int z, float val[3])
					{
						int index = z*(ydim*xdim) + y*xdim + x;
						xd[index] = val[0];
						yd[index] = val[1];
						zd[index] = val[2];
					}

					void QuadMesh3D::VectorData::WriteFile(DBfile *db, const char *meshName)
					{
						// Create subvar names.
						std::string xvar(name + std::string("_X"));
						std::string yvar(name + std::string("_Y"));
						std::string zvar(name + std::string("_Z"));
						char *varnames[3];
						varnames[0] = (char *)xvar.c_str();
						varnames[1] = (char *)yvar.c_str();
						varnames[2] = (char *)zvar.c_str();
						float *vars[] = { xd, yd, zd };

						DBoptlist *optList = DBMakeOptlist(2);
						DBAddOption(optList, DBOPT_UNITS, (void*)"cm/s");
						if (nodal)
						{
							int ndims[] = { xdim, ydim, zdim };
							DBPutQuadvar(db, (char *)name.c_str(), (char *)meshName,
								3, varnames, vars, ndims, 3, NULL, 0, DB_FLOAT,
								DB_NODECENT, optList);
						}
						else
						{
							int zdims[] = { xdim - 1, ydim - 1, zdim - 1 };
							DBPutQuadvar(db, (char *)name.c_str(), (char *)meshName,
								3, varnames, vars, zdims, 3, NULL, 0, DB_FLOAT,
								DB_ZONECENT, optList);
						}
						DBFreeOptlist(optList);
					}


					QuadMesh3D::ScalarData::ScalarData(const std::string &n, int nx, int ny, int nz, bool node) : name(n)
					{
						xdim = nx;
						ydim = ny;
						zdim = nz;
						int ndata;
						if (node)
							ndata = xdim * ydim * zdim;
						else
							ndata = (xdim - 1) * (ydim - 1) * (zdim - 1);
						data = new float[ndata];
						nodal = node;
					}

					QuadMesh3D::ScalarData::~ScalarData()
					{
						delete[] data;
					}

					int QuadMesh3D::ScalarData::ZonalIndex(int x, int y, int z) const
					{
						return z*((ydim - 1)*(xdim - 1)) + y*(xdim - 1) + x;
					}

					int QuadMesh3D::ScalarData::NodalIndex(int x, int y, int z) const
					{
						return z*(ydim*xdim) + y*xdim + x;
					}

					void QuadMesh3D::ScalarData::SetZonalValue(int x, int y, int z, float val)
					{
						data[ZonalIndex(x, y, z)] = val;
					}

					void QuadMesh3D::ScalarData::SetNodalValue(int x, int y, int z, float val)
					{
						data[NodalIndex(x, y, z)] = val;
					}

					void QuadMesh3D::ScalarData::WriteFile(DBfile *db, const char *meshName)
					{
						DBoptlist *optList = DBMakeOptlist(2);
						DBAddOption(optList, DBOPT_UNITS, (void*)"Joules");
						if (nodal)
						{
							int ndims[] = { xdim, ydim, zdim };
							DBPutQuadvar1(db, (char *)name.c_str(), (char *)meshName, data,
								ndims, 3, NULL, 0, DB_FLOAT, DB_NODECENT, optList);
						}
						else
						{
							int zdims[] = { xdim - 1, ydim - 1, zdim - 1 };
							DBPutQuadvar1(db, (char *)name.c_str(), (char *)meshName, data,
								zdims, 3, NULL, 0, DB_FLOAT, DB_ZONECENT, optList);
						}
						DBFreeOptlist(optList);
					}

					void QuadMesh3D::ScalarData::WriteDataSlice(DBfile *db, const std::string &newMeshName,
						const std::string &newVarName, int sliceVal, int sliceDimension)
					{
						DBoptlist *optList = DBMakeOptlist(1);
						DBAddOption(optList, DBOPT_UNITS, (void*)"Joules");
						float *sliceData;

						if (nodal)
						{
							int ndata, index = 0;
							int ndims[2];
							if (sliceDimension == 0)
							{
								ndata = zdim * ydim;
								ndims[0] = zdim; ndims[1] = ydim;
								sliceData = new float[ndata];
								for (int y = 0; y < ydim; ++y)
								for (int z = 0; z < zdim; ++z)
								{
									int originalIndex = NodalIndex(sliceVal, y, z);
									sliceData[index++] = data[originalIndex];
								}
							}
							else if (sliceDimension == 1)
							{
								ndata = zdim * xdim;
								sliceData = new float[ndata];
								ndims[0] = xdim; ndims[1] = zdim;
								for (int z = 0; z < zdim; ++z)
								for (int x = 0; x < xdim; ++x)
								{
									int originalIndex = NodalIndex(x, sliceVal, z);
									sliceData[index++] = data[originalIndex];
								}
							}
							else
							{
								ndata = xdim * ydim;
								sliceData = new float[ndata];
								ndims[0] = xdim; ndims[1] = ydim;
								for (int y = 0; y < ydim; ++y)
								for (int x = 0; x < xdim; ++x)
								{
									int originalIndex = NodalIndex(x, y, sliceVal);
									sliceData[index++] = data[originalIndex];
								}
							}

							DBPutQuadvar1(db, (char *)newVarName.c_str(), (char *)newMeshName.c_str(),
								sliceData, ndims, 2, NULL, 0, DB_FLOAT, DB_NODECENT, optList);

							delete[] sliceData;
						}
						else
						{
							int ndata, index = 0;
							int zdims[2];
							if (sliceDimension == 0)
							{
								ndata = (zdim - 1) * (ydim - 1);
								zdims[0] = zdim - 1; zdims[1] = ydim - 1;
								sliceData = new float[ndata];
								for (int y = 0; y < ydim - 1; ++y)
								for (int z = 0; z < zdim - 1; ++z)
								{
									int originalIndex = ZonalIndex(sliceVal, y, z);
									sliceData[index++] = data[originalIndex];
								}
							}
							else if (sliceDimension == 1)
							{
								ndata = (zdim - 1) * (xdim - 1);
								sliceData = new float[ndata];
								zdims[0] = xdim - 1; zdims[1] = zdim - 1;
								for (int z = 0; z < zdim - 1; ++z)
								for (int x = 0; x < xdim - 1; ++x)
								{
									int originalIndex = ZonalIndex(x, sliceVal, z);
									sliceData[index++] = data[originalIndex];
								}
							}
							else
							{
								ndata = (xdim - 1) * (ydim - 1);
								sliceData = new float[ndata];
								zdims[0] = xdim - 1; zdims[1] = ydim - 1;
								for (int y = 0; y < ydim - 1; ++y)
								for (int x = 0; x < xdim - 1; ++x)
								{
									int originalIndex = ZonalIndex(x, y, sliceVal);
									sliceData[index++] = data[originalIndex];
								}
							}

							DBPutQuadvar1(db, (char *)newVarName.c_str(), (char *)newMeshName.c_str(),
								sliceData, zdims, 2, NULL, 0, DB_FLOAT, DB_ZONECENT, optList);

							delete[] sliceData;
						}
						DBFreeOptlist(optList);
					}

void QuadMesh3D::ScalarData::ZonalGradientAt(int i, int j, int k, float grad[3]) const
					{
						if (i == 0)
							grad[0] = (data[ZonalIndex(i + 1, j, k)] - data[ZonalIndex(i, j, k)]);
						else if (i == xdim - 2)
							grad[0] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i - 1, j, k)]);
						else
							grad[0] = (data[ZonalIndex(i + 1, j, k)] - data[ZonalIndex(i - 1, j, k)]) * 0.5f;

						if (j == 0)
							grad[1] = (data[ZonalIndex(i, j + 1, k)] - data[ZonalIndex(i, j, k)]);
						else if (j == ydim - 2)
							grad[1] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i, j - 1, k)]);
						else
							grad[1] = (data[ZonalIndex(i, j + 1, k)] - data[ZonalIndex(i, j - 1, k)]) * 0.5f;

						if (k == 0)
							grad[2] = (data[ZonalIndex(i, j, k + 1)] - data[ZonalIndex(i, j, k)]);
						else if (k == zdim - 2)
							grad[2] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i, j, k - 1)]);
						else
							grad[2] = (data[ZonalIndex(i, j, k + 1)] - data[ZonalIndex(i, j, k - 1)]) * 0.5f;
					}

void QuadMesh3D::ScalarData::NodalGradientAt(int i, int j, int k, float grad[3]) const
					{
						if (i == 0)
							grad[0] = (data[NodalIndex(i + 1, j, k)] - data[NodalIndex(i, j, k)]);
						else if (i == xdim - 1)
							grad[0] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i - 1, j, k)]);
						else
							grad[0] = (data[NodalIndex(i + 1, j, k)] - data[NodalIndex(i - 1, j, k)]) * 0.5f;

						if (j == 0)
							grad[1] = (data[NodalIndex(i, j + 1, k)] - data[NodalIndex(i, j, k)]);
						else if (j == ydim - 1)
							grad[1] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i, j - 1, k)]);
						else
							grad[1] = (data[NodalIndex(i, j + 1, k)] - data[NodalIndex(i, j - 1, k)]) * 0.5f;

						if (k == 0)
							grad[2] = (data[NodalIndex(i, j, k + 1)] - data[NodalIndex(i, j, k)]);
						else if (k == zdim - 1)
							grad[2] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i, j, k - 1)]);
						else
							grad[2] = (data[NodalIndex(i, j, k + 1)] - data[NodalIndex(i, j, k - 1)]) * 0.5f;
					}

void QuadMesh3D::ScalarData::ZonalTensorGradientAt(int i, int j, int k, float grad[9]) const
					{
						// ii
						if (i == 0)
							grad[0] = (data[ZonalIndex(i + 1, j, k)] - data[ZonalIndex(i, j, k)]);
						else if (i == xdim - 2)
							grad[0] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i - 1, j, k)]);
						else
							grad[0] = (data[ZonalIndex(i + 1, j, k)] - data[ZonalIndex(i - 1, j, k)]) * 0.5f;

						// ij
						if ((i == 0) || (j == 0) || (i == xdim - 2) || (j == ydim - 2))
							grad[1] = 0.;
						else
							grad[1] = (data[ZonalIndex(i - 1, j - 1, k)] - data[ZonalIndex(i + 1, j + 1, k)]) * 0.5f;

						// ik
						if ((i == 0) || (k == 0) || (i == xdim - 2) || (k == zdim - 2))
							grad[2] = 0.;
						else
							grad[2] = (data[ZonalIndex(i - 1, j, k - 1)] - data[ZonalIndex(i + 1, j, k + 1)]) * 0.5f;

						// ji
						grad[3] = grad[1];

						// jj
						if (j == 0)
							grad[4] = (data[ZonalIndex(i, j + 1, k)] - data[ZonalIndex(i, j, k)]);
						else if (j == ydim - 2)
							grad[4] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i, j - 1, k)]);
						else
							grad[4] = (data[ZonalIndex(i, j + 1, k)] - data[ZonalIndex(i, j - 1, k)]) * 0.5f;

						// jk
						if ((j == 0) || (k == 0) || (j == ydim - 2) || (k == zdim - 2))
							grad[5] = 0.;
						else
							grad[5] = (data[ZonalIndex(i, j - 1, k - 1)] - data[ZonalIndex(i, j + 1, k + 1)]) * 0.5f;

						// ki
						grad[6] = grad[2];

						// kj
						grad[7] = grad[5];

						// kk
						if (k == 0)
							grad[8] = (data[ZonalIndex(i, j, k + 1)] - data[ZonalIndex(i, j, k)]);
						else if (k == zdim - 2)
							grad[8] = (data[ZonalIndex(i, j, k)] - data[ZonalIndex(i, j, k - 1)]);
						else
							grad[8] = (data[ZonalIndex(i, j, k + 1)] - data[ZonalIndex(i, j, k - 1)]) * 0.5f;
					}

void QuadMesh3D::ScalarData::NodalTensorGradientAt(int i, int j, int k, float grad[9]) const
					{
						// ii
						if (i == 0)
							grad[0] = (data[NodalIndex(i + 1, j, k)] - data[NodalIndex(i, j, k)]);
						else if (i == xdim - 1)
							grad[0] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i - 1, j, k)]);
						else
							grad[0] = (data[NodalIndex(i + 1, j, k)] - data[NodalIndex(i - 1, j, k)]) * 0.5f;

						// ij
						if ((i == 0) || (j == 0) || (i == xdim - 1) || (j == ydim - 1))
							grad[1] = 0.;
						else
							grad[1] = (data[NodalIndex(i - 1, j - 1, k)] - data[NodalIndex(i + 1, j + 1, k)]) * 0.5f;

						// ik
						if ((i == 0) || (k == 0) || (i == xdim - 1) || (k == zdim - 1))
							grad[2] = 0.;
						else
							grad[2] = (data[NodalIndex(i - 1, j, k - 1)] - data[NodalIndex(i + 1, j, k + 1)]) * 0.5f;

						// ji
						grad[3] = grad[1];

						// jj
						if (j == 0)
							grad[4] = (data[NodalIndex(i, j + 1, k)] - data[NodalIndex(i, j, k)]);
						else if (j == ydim - 1)
							grad[4] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i, j - 1, k)]);
						else
							grad[4] = (data[NodalIndex(i, j + 1, k)] - data[NodalIndex(i, j - 1, k)]) * 0.5f;

						// jk
						if ((j == 0) || (k == 0) || (j == ydim - 1) || (k == zdim - 1))
							grad[5] = 0.;
						else
							grad[5] = (data[NodalIndex(i, j - 1, k - 1)] - data[NodalIndex(i, j + 1, k + 1)]) * 0.5f;

						// ki
						grad[6] = grad[2];

						// kj
						grad[7] = grad[5];

						// kk
						if (k == 0)
							grad[8] = (data[NodalIndex(i, j, k + 1)] - data[NodalIndex(i, j, k)]);
						else if (k == zdim - 1)
							grad[8] = (data[NodalIndex(i, j, k)] - data[NodalIndex(i, j, k - 1)]);
						else
							grad[8] = (data[NodalIndex(i, j, k + 1)] - data[NodalIndex(i, j, k - 1)]) * 0.5f;
					}

QuadMesh3D::VectorData * QuadMesh3D::ScalarData::CreateGradient(const char *name)
					{
						VectorData *vec = new VectorData(name, xdim, ydim, zdim, nodal);
						float grad[3];

						if (!nodal)
						{
							// Create the data.
							for (int i = 0; i < xdim - 1; ++i)
							for (int j = 0; j < ydim - 1; ++j)
							for (int k = 0; k < zdim - 1; ++k)
							{
								ZonalGradientAt(i, j, k, grad);

								// If the zonal gradient is zero then make it a small
								// gradient in the up direction.
								if (grad[0] == 0. && grad[1] == 0. && grad[2] == 0.)
								{
									grad[1] = 0.01f;
								}

								vec->SetZonalValue(i, j, k, grad);
							}
						}
						else
						{
							// Create the data.
							for (int i = 0; i < xdim; ++i)
							for (int j = 0; j < ydim; ++j)
							for (int k = 0; k < zdim; ++k)
							{
								NodalGradientAt(i, j, k, grad);
								vec->SetNodalValue(i, j, k, grad);
							}
						}

						return vec;
					}

QuadMesh3D::TensorData * QuadMesh3D::ScalarData::CreateGradientTensor(const char *name)
					{
						ScalarData *sub_comps[9];

						for (int i = 0; i < 9; i++)
						{
							char comp1;
							char comp2;
							switch (i % 3)
							{
							case 0: comp1 = 'i'; break;
							case 1: comp1 = 'j'; break;
							default: comp1 = 'k'; break;
							}
							switch (i / 3)
							{
							case 0: comp2 = 'i'; break;
							case 1: comp2 = 'j'; break;
							default: comp2 = 'k'; break;
							}
							char comp_name[1024];
							sprintf(comp_name, "tensor_comps/%s_%c%c", name, comp1, comp2);
							sub_comps[i] = new ScalarData(comp_name, xdim, ydim, zdim, nodal);
						}
						if (!nodal)
						{
							// Create the data.
							for (int i = 0; i < xdim - 1; ++i)
							for (int j = 0; j < ydim - 1; ++j)
							for (int k = 0; k < zdim - 1; ++k)
							{
								float vals[9];
								ZonalTensorGradientAt(i, j, k, vals);
								for (int l = 0; l < 9; l++)
									sub_comps[l]->SetZonalValue(i, j, k, vals[l]);
							}
						}
						else
						{
							// Create the data.
							for (int i = 0; i < xdim; ++i)
							for (int j = 0; j < ydim; ++j)
							for (int k = 0; k < zdim; ++k)
							{
								float vals[9];
								NodalTensorGradientAt(i, j, k, vals);
								for (int l = 0; l < 9; l++)
									sub_comps[l]->SetNodalValue(i, j, k, vals[l]);
							}
						}

						return new TensorData(name, sub_comps);
					}

					QuadMesh3D::TensorData::TensorData(const std::string &n, ScalarData *comps[9]) : name(n)
					{
						for (int i = 0; i < 9; i++)
							components[i] = comps[i];
					}

					QuadMesh3D::TensorData::~TensorData()
					{
						for (int i = 0; i < 9; i++)
							delete components[i];
					}

					void QuadMesh3D::TensorData::WriteFile(DBfile *db, const char *meshName)
					{
						DBMkDir(db, "tensor_comps");
						char absolute_meshname[1024];
						sprintf(absolute_meshname, "/%s", meshName);
						for (int i = 0; i < 9; i++)
							components[i]->WriteFile(db, absolute_meshname);
						char defvars[1024];
						sprintf(defvars, "%s tensor { { <%s>, <%s>, <%s> }, { <%s>, <%s>, <%s> }, { <%s>, <%s>, <%s> } }; %s_diagonal array array_compose(<%s>, <%s>, <%s>)",
							name.c_str(), components[0]->GetName().c_str(),
							components[1]->GetName().c_str(), components[2]->GetName().c_str(),
							components[3]->GetName().c_str(), components[4]->GetName().c_str(),
							components[5]->GetName().c_str(), components[6]->GetName().c_str(),
							components[7]->GetName().c_str(), components[8]->GetName().c_str(),
							name.c_str(), components[0]->GetName().c_str(),
							components[4]->GetName().c_str(), components[8]->GetName().c_str());
						int len = (int) strlen(defvars) + 1;
						DBWrite(db, "_visit_defvars", defvars, &len, 1, DB_CHAR);
					}

					QuadMesh3D::SliceInfo::SliceInfo(const std::string &nm, const std::string &nvn,
						int sv, int sd, QuadMesh3D::ScalarData *d) : newMeshName(nm), newVarName(nvn)
					{
						sliceVal = sv;
						sliceDimension = sd;
						scalars = d;
					}

					QuadMesh3D::SliceInfo::~SliceInfo()
					{
					}

					void QuadMesh3D::SliceInfo::WriteFile(DBfile *db, QuadMesh3D *qm)
					{
						// Write a slice of the mesh
						qm->WriteMeshSlice(db, newMeshName, sliceVal, sliceDimension);

						// Write the sliced data on that mesh.
						scalars->WriteDataSlice(db, newMeshName, newVarName, sliceVal, sliceDimension);
					}


					QuadMesh3D::QuadMesh3D(int nx, int ny, int nz, bool rect) : scalarData(),
					vectorData(), tensorData(), sliceInfo(), meshName("Mesh")
				{
					xdim = nx;
					ydim = ny;
					zdim = nz;
					if (rect)
					{
						coordX = new float[nx];
						coordY = new float[ny];
						coordZ = new float[nz];
					}
					else
					{
						int nels = xdim * ydim * zdim;
						coordX = new float[nels];
						coordY = new float[nels];
						coordZ = new float[nels];
					}

					// Allocate storage for the material cells.
					mats.AllocClean((xdim - 1) * (ydim - 1) * (zdim - 1));
					writeMaterial = false;
				}

					QuadMesh3D::~QuadMesh3D()
				{
					delete[] coordX;
					delete[] coordY;
					delete[] coordZ;

					int i;
					for (i = 0; i < scalarData.size(); ++i)
						delete scalarData[i];
					for (i = 0; i < vectorData.size(); ++i)
						delete vectorData[i];
					for (i = 0; i < tensorData.size(); ++i)
						delete tensorData[i];
					for (i = 0; i < sliceInfo.size(); ++i)
						delete sliceInfo[i];
				}

					void QuadMesh3D::SetMeshName(const std::string &name)
				{
					meshName = name;
				}

					void QuadMesh3D::CreateZonalData(const char *name, float(*zonal)(int, int, int, QuadMesh3D *))
				{
					ScalarData *m = new ScalarData(name, xdim, ydim, zdim, false);
					scalarData.push_back(m);

					// Create the data.
					for (int i = 0; i < xdim - 1; ++i)
					for (int j = 0; j < ydim - 1; ++j)
					for (int k = 0; k < zdim - 1; ++k)
						m->SetZonalValue(i, j, k, (*zonal)(i, j, k, this));
				}

					void QuadMesh3D::CreateNodalData(const char *name, float(*nodal)(float *, QuadMesh3D *))
				{
					ScalarData *m = new ScalarData(name, xdim, ydim, zdim, true);
					scalarData.push_back(m);

					// Create the data.
					for (int i = 0; i < xdim; ++i)
					for (int j = 0; j < ydim; ++j)
					for (int k = 0; k < zdim; ++k)
					{
						float pt[3];
						pt[0] = GetX(i, j, k);
						pt[1] = GetY(i, j, k);
						pt[2] = GetZ(i, j, k);
						m->SetNodalValue(i, j, k, (*nodal)(pt, this));
					}
				}

					void QuadMesh3D::CreateNodalVectorData(const char *name, void(*nodal)(float *, int, int, int, QuadMesh3D *))
				{
					VectorData *m = new VectorData(name, xdim, ydim, zdim, true);
					vectorData.push_back(m);

					// Create the data.
					for (int i = 0; i < xdim; ++i)
					for (int j = 0; j < ydim; ++j)
					for (int k = 0; k < zdim; ++k)
					{
						float vec[3];
						// Get the vector value.
						(*nodal)(vec, i, j, k, this);
						m->SetNodalValue(i, j, k, vec);
					}
				}

					void QuadMesh3D::CreateGradient(const char *name, const char *gradName)
				{
					for (int i = 0; i < scalarData.size(); ++i)
					{
						if (scalarData[i]->GetName() == name)
						{
							VectorData *gradient = scalarData[i]->CreateGradient(gradName);
							vectorData.push_back(gradient);
							break;
						}
					}
				}

					void QuadMesh3D::CreateGradientTensor(const char *varName, const char *outputName)
				{
					for (int i = 0; i < scalarData.size(); ++i)
					{
						if (scalarData[i]->GetName() == varName)
						{
							TensorData *tensor = scalarData[i]->CreateGradientTensor(outputName);
							tensorData.push_back(tensor);
							break;
						}
					}
				}

					void QuadMesh3D::AddMaterial(const char *matname)
				{
					mats.AddMaterial(matname);
				}

					void QuadMesh3D::CreateMaterialData(void(*createmat)(int, int, int, int *, double*, int *, QuadMesh3D *))
				{
					int nMats = 1;
					int matnos[100];
					double matVf[100];

					// Create the data.
					for (int i = 0; i < xdim - 1; ++i)
					for (int j = 0; j < ydim - 1; ++j)
					for (int k = 0; k < zdim - 1; ++k)
					{
						int zoneid = k*((ydim - 1)*(xdim - 1)) + j*(xdim - 1) + i;
						(*createmat)(i, j, k, matnos, matVf, &nMats, this);
						if (nMats > 1)
							mats.AddMixed(zoneid, matnos, matVf, nMats);
						else
							mats.AddClean(zoneid, matnos[0]);
					}

					writeMaterial = true;
				}

					void QuadMesh3D::AddSlice(const std::string &varName, const std::string &nmn, const std::string &nvn,
					int sliceVal, int sliceDimension)
				{
					// Look for the variable name in the scalars array.
					for (int i = 0; i < scalarData.size(); ++i)
					{
						if (scalarData[i]->GetName() == varName)
						{
							SliceInfo *slice = new SliceInfo(nmn, nvn, sliceVal, sliceDimension, scalarData[i]);
							sliceInfo.push_back(slice);
							return;
						}
					}

					std::cerr << "The variable " << varName.c_str() << " is not a scalar variable." << std::endl;
				}

					void QuadMesh3D::WriteFile(DBfile *db)
				{
					// Write the mesh.
					WriteMesh(db);

					// Write the scalar mesh data
					int i;
					for (i = 0; i < scalarData.size(); ++i)
						scalarData[i]->WriteFile(db, meshName.c_str());

					// Write the vector mesh data
					for (i = 0; i < vectorData.size(); ++i)
						vectorData[i]->WriteFile(db, meshName.c_str());

					// Write the tensor mesh data
					for (i = 0; i < tensorData.size(); ++i)
						tensorData[i]->WriteFile(db, meshName.c_str());

					// Write the material data
					if (writeMaterial)
						mats.WriteMaterial(db, "mat1", meshName.c_str(), xdim - 1, ydim - 1, zdim - 1);

					// Write the slices
					for (i = 0; i < sliceInfo.size(); ++i)
						sliceInfo[i]->WriteFile(db, this);
				}

		}
	}
}


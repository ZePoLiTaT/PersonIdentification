// cJSON
#include "cJSON.h"

// OMLT
#include "Model.h"

#include "MultilayerPerceptron.h"


namespace OMLT
{
	bool Model::FromJSON(const std::string& in_json, Model& out_model)
	{
		bool result = false;
		out_model.type = ModelType::Invalid;
		out_model.ptr = nullptr;

		cJSON* cj_root = cJSON_Parse(in_json.c_str());
		if(cj_root)
		{
			if(cJSON* cj_type = cJSON_GetObjectItem(cj_root, "Type"))
			{
				
				if(strcmp(cj_type->valuestring, "MultilayerPerceptron") == 0)
				{
					if(MLP* mlp = MLP::FromJSON(cj_root))
					{
						out_model.type = ModelType::MLP;
						out_model.mlp = mlp;
						result = true;
					}
				}
				
			}
		}

		cJSON_Delete(cj_root);
		return result;
	}
}
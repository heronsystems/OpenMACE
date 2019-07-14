/* MyMEXFunction
 * Adds second input to each  
 * element of first input
 * a = MyMEXFunction(xpts,ypts,Cinv,measurements,ax,ay);
*/

#include "mex.hpp"
#include "mexAdapter.hpp"

using namespace matlab::data;
using matlab::mex::ArgumentList;

class MexFunction : public matlab::mex::Function {
public:
    void operator()(ArgumentList outputs, ArgumentList inputs) {
        checkArguments(outputs, inputs);
        // compute 
//         matlab::data::TypedArray<double> ax = inputs[4];
         double ax = inputs[4][0];
         double ay = inputs[5][0];
         int nx = inputs[0].getNumberOfElements();
         int ny = inputs[1].getNumberOfElements();
//         matlab::data::TypedArray<double> Cinv = inputs[3];
         Reference<Array> Cinvref = &inputs[3];
        std::cout << "nx :" << nx << std::endl;
        std::cout << "ny :" << ny << std::endl;
//        std::cout << "nm :" << Cinv.size() << std::endl;
        std::cout << "ax :" << ax << std::endl;
        std::cout << "ay :" << ay << std::endl;
         for (int i=0; i<nx; i++){
           for (int j=0; i<ny; i++){

           }        
         }

//        double* ay=inputs[6].release().get();

        //double ax = inputs[5];
        //double ay = inputs[6];
//        std::cout << "(ax,ay)" << ax[0] << "," << ay;

//.        const double offSet = inputs[0][0];

//         TypedArray<double> doubleArray = std::move(inputs[1]);
//         for (auto& elem : doubleArray) {
//             elem += offSet;
//         }
//         outputs[0] = doubleArray;
    }

    void checkArguments(ArgumentList outputs, ArgumentList inputs) {
        std::cout << "test\n";
        
        // Get pointer to engine
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

        // Get array factory
        ArrayFactory factory;

        // Check first input argument
        if (inputs[0].getType() != ArrayType::DOUBLE ||
            inputs[0].getNumberOfElements() == 0)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("First input must be nonempty vector (ypts)") }));
        }
        
        // Check second input argument
        if (inputs[1].getType() != ArrayType::DOUBLE ||
            inputs[1].getNumberOfElements() == 0)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Second input must be nonempty vector (xpts)") }));
        }
        
        // Check third input argument
        if (inputs[2].getType() != ArrayType::DOUBLE ||
            inputs[2].getNumberOfElements() == 0)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Third input must be nonempty matrix (Cinv)") }));
        }      

        // Check fourth input argument
        if (inputs[3].getType() != ArrayType::DOUBLE ||
            inputs[3].getNumberOfElements() == 0)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Third input must be nonempty matrix (measurements)") }));
        }      
        
        // Check fifth input argument
        if (inputs[4].getType() != ArrayType::DOUBLE ||
            inputs[4].getNumberOfElements() != 1)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Four input must be scalar (ax)") }));
        }       
        
        // Check four input argument
        if (inputs[5].getType() != ArrayType::DOUBLE ||
            inputs[5].getNumberOfElements() != 1)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Fifth input must be scalar (ay)") }));
        }               

        // Check number of outputs
        if (outputs.size() > 1) {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Only one output is returned") }));
        }
        
    }
};

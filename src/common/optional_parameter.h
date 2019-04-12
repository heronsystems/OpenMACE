#ifndef OPTIONAL_PARAMETER_H
#define OPTIONAL_PARAMETER_H


template <typename T>
class OptionalParameter
{
public:

    OptionalParameter() :
        mIsSet(false)
    {

    }

    OptionalParameter(const T &data) :
        mIsSet(true),
        mdata(data)
    {

    }

    bool IsSet() const {
        return mIsSet;
    }

    T Value() const
    {
        return mdata;
    }

    T operator()() const {
        return mdata;
    }

private:

    bool mIsSet;
    T mdata;
};

#endif // OPTIONAL_PARAMETER_H

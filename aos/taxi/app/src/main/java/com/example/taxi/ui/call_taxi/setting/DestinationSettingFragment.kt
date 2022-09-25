package com.example.taxi.ui.call_taxi.setting

import android.util.Log
import android.view.View
import androidx.appcompat.widget.SearchView
import androidx.core.os.bundleOf
import androidx.fragment.app.viewModels
import androidx.navigation.fragment.findNavController
import androidx.recyclerview.widget.LinearLayoutManager
import androidx.recyclerview.widget.RecyclerView
import com.example.taxi.R
import com.example.taxi.base.BaseFragment
import com.example.taxi.data.api.KakaoAPI
import com.example.taxi.data.dto.mypage.Favorites
import com.example.taxi.data.dto.user.destination.Destination
import com.example.taxi.data.dto.user.destination.FrequentDestination
import com.example.taxi.data.dto.user.destination.DestinationSearch
import com.example.taxi.data.dto.user.destination.DestinationSearchDto
import com.example.taxi.databinding.FragmentDestinationSettingBinding
import com.example.taxi.ui.home.user.DestinationListAdapter
import com.example.taxi.ui.home.user.FavoritesAdapter
import com.example.taxi.ui.home.user.FavoritesDialogFragment
import com.example.taxi.ui.home.user.UserHomeViewModel
import com.example.taxi.utils.constant.KakaoApi
import com.example.taxi.utils.constant.UiState
import com.example.taxi.utils.constant.hide
import com.example.taxi.utils.constant.show
import com.example.taxi.utils.view.toast
import dagger.hilt.android.AndroidEntryPoint
import retrofit2.Call
import retrofit2.Callback
import retrofit2.Response
import retrofit2.Retrofit
import retrofit2.converter.gson.GsonConverterFactory
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.roundToInt
import kotlin.math.sqrt

@AndroidEntryPoint
class DestinationSettingFragment : BaseFragment<FragmentDestinationSettingBinding>(R.layout.fragment_destination_setting) {

    var checkState = true
    private lateinit var startingPoint : Destination
    private lateinit var destination : Destination
    private lateinit var destinationListAdapter: DestinationListAdapter
    private lateinit var favoritesAdapter: FavoritesAdapter
    private lateinit var destinationSearchListAdapter: DestinationSearchListAdapter
    private val userHomeViewModel : UserHomeViewModel by viewModels()

    private val destinationOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        binding.textDestinationSettingDestination.text = destination.addressName
        checkEnd()
    }

    private val favoritesOnClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        destination = Destination(address,x,place,y)
        binding.textDestinationSettingDestination.text = destination.addressName
        checkEnd()
    }

    private val destinationSearchClickListener: (View, String, String, String, String) -> Unit = { _, place, address, x, y ->
        binding.searchDestinationSetting.visibility = View.GONE
        binding.recyclerviewDestinationSettingSearch.visibility = View.GONE
        binding.textDestinationSettingStart.visibility = View.VISIBLE
        binding.imageDestinationSettingForward.visibility = View.VISIBLE
        binding.textDestinationSettingDestination.visibility = View.VISIBLE
        binding.searchDestinationSetting.setQuery("", false)
        destination = Destination(address,x,place,y)
        binding.textDestinationSettingDestination.text = destination.addressName
        checkEnd()
    }

    private fun checkEnd(){
        if(binding.textDestinationSettingStart.text !="" && binding.textDestinationSettingDestination.text != ""){
            findNavController().navigate(R.id.action_destinationSettingFragment_to_callTaxiFragment,
            bundleOf("Destination" to destination, "StartingPoint" to startingPoint))
        }
    }

    private val favoritesDeleteClickListener: (View, String) -> Unit = { _, address ->
        showFavoritesDialog(address)
    }

    override fun init() {
        initData()
        initAdapter()
        observerData()
        setOnClickListeners()
    }

    private fun initData() {
        if(arguments?.getParcelable<Destination>("Destination")!=null){
            destination = arguments?.getParcelable<Destination>("Destination") as Destination
            binding.textDestinationSettingDestination.text = destination.addressName
        }
        if(arguments?.getParcelable<Destination>("StartingPoint")!=null){
            startingPoint = arguments?.getParcelable<Destination>("StartingPoint") as Destination
            binding.textDestinationSettingStart.text = startingPoint.addressName
        }
    }

    private fun initAdapter() {
        userHomeViewModel.getDestinations()
        destinationListAdapter = DestinationListAdapter().apply {
            onItemClickListener = destinationOnClickListener
        }
        binding.recyclerviewDestinationSettingDestinationList.layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        binding.recyclerviewDestinationSettingDestinationList.adapter = destinationListAdapter
        userHomeViewModel.getFavorites()
        favoritesAdapter = FavoritesAdapter().apply {
            onItemClickListener = favoritesDeleteClickListener
            onFavoritesClickListener = favoritesOnClickListener
        }
        binding.recyclerviewDestinationSettingFavorites.apply {
            adapter = favoritesAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
    }

    private fun observerData() {
        userHomeViewModel.destinations.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewDestinationSettingDestinationList.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textDestinationSettingNoContentDestination.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    val list : MutableList<FrequentDestination> = state.data.toMutableList()
                    destinationListAdapter.updateList(list)
                    binding.recyclerviewDestinationSettingDestinationList.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textDestinationSettingNoContentDestination.hide()

                }
            }
        }
        userHomeViewModel.favorites.observe(viewLifecycleOwner) { state ->
            when (state) {
                is UiState.Loading -> {
                    //binding.progressBar.show()
                }
                is UiState.Failure -> {
                    //binding.progressBar.hide()
                    state.error?.let {
                        toast(it)
                        Log.d("UiState.Failure", it)
                    }
                    binding.recyclerviewDestinationSettingFavorites.setBackgroundResource(R.drawable.layout_recycler_no_item)
                    binding.textDestinationSettingNoContentFavorites.show()
                }
                is UiState.Success -> {
                    //binding.progressBar.hide()
                    val list : MutableList<Favorites> = state.data.toMutableList()
                    favoritesAdapter.updateList(list)
                    binding.recyclerviewDestinationSettingFavorites.setBackgroundResource(R.drawable.layout_recycler)
                    binding.textDestinationSettingNoContentFavorites.hide()
                }
            }
        }
    }

    private fun setOnClickListeners() {
        binding.textDestinationSettingStart.setOnClickListener {
            if(binding.textDestinationSettingDestination.text.toString()!=""){
                findNavController().navigate(R.id.action_destinationSettingFragment_to_startPointSettingFragment
                    ,bundleOf("Destination" to destination))
            }else{
                findNavController().navigate(R.id.action_destinationSettingFragment_to_startPointSettingFragment)
            }
        }
        binding.textDestinationSettingDestination.setOnClickListener {
            binding.searchDestinationSetting.visibility = View.VISIBLE
            binding.textDestinationSettingStart.visibility = View.GONE
            binding.imageDestinationSettingForward.visibility = View.GONE
            binding.textDestinationSettingDestination.visibility = View.GONE
            checkState = true
        }
        binding.searchDestinationSetting.setOnQueryTextListener(object : SearchView.OnQueryTextListener {
            override fun onQueryTextSubmit(query: String?): Boolean {
                // 검색 버튼 누를 때 호출

                return true
            }

            override fun onQueryTextChange(newText: String?): Boolean {
                if (newText != null && newText != "") {
                    searchKeyword(newText)
                }
                return true
            }
        })
        binding.imageDestinationSettingBack.setOnClickListener {
            requireActivity().onBackPressed()
        }
    }

    private fun searchKeyword(keyword: String) {
        val retrofit = Retrofit.Builder()   // Retrofit 구성
            .baseUrl(KakaoApi.BASE_URL)
            .addConverterFactory(GsonConverterFactory.create())
            .build()
        val api = retrofit.create(KakaoAPI::class.java)   // 통신 인터페이스를 객체로 생성
        val call = api.getSearchKeyword(KakaoApi.API_KEY, keyword)   // 검색 조건 입력

        // API 서버에 요청
        call.enqueue(object: Callback<DestinationSearchDto> {
            override fun onResponse(
                call: Call<DestinationSearchDto>,
                response: Response<DestinationSearchDto>
            ) {
                // 통신 성공 (검색 결과는 response.body()에 담겨있음)
                Log.d("Test", "Raw: ${response.raw()}")
                Log.d("Test", "Body: ${response.body()}")
                if(response.body()!!.documents != null){
                    if(checkState){
                        getDistance(response.body()!!.documents)
                    }else{
                        initSearchAdapter(response.body()!!.documents)
                    }
                }
            }

            override fun onFailure(call: Call<DestinationSearchDto>, t: Throwable) {
                // 통신 실패
                Log.w("MainActivity", "통신 실패: ${t.message}")
            }
        })
    }

    private fun getDistance(list : List<DestinationSearch>) {
        if(startingPoint!=null){
            for(i in list){
                var newX = ( kotlin.math.cos(startingPoint.latitude.toDouble()) * 6400 * 2 * 3.14 / 360 ) * abs(startingPoint.longitude.toDouble() - i.x.toDouble())
                var newY = 111 * abs(startingPoint.longitude.toDouble() - i.y.toDouble())

                i.distance = ((sqrt(newX.pow(2)+newY.pow(2)) * 100.0).roundToInt() / 100.0).toString()+"Km"
            }
        }
        initSearchAdapter(list)
    }

    private fun initSearchAdapter(list : List<DestinationSearch>){
        destinationSearchListAdapter = DestinationSearchListAdapter().apply {
            onItemClickListener = destinationSearchClickListener
        }
        binding.recyclerviewDestinationSettingSearch.apply {
            adapter = destinationSearchListAdapter
            layoutManager = LinearLayoutManager(requireContext(), RecyclerView.VERTICAL, false)
        }
        destinationSearchListAdapter.updateList(list)
        binding.recyclerviewDestinationSettingSearch.visibility = View.VISIBLE
    }

    private fun showFavoritesDialog(address: String) {
        FavoritesDialogFragment { favoritesListener(address) }.show(childFragmentManager, "FAVORITES_DIALOG")
    }

    private val favoritesListener: (address: String) -> Unit = {
        //userHomeViewModel.deleteFavorites()
    }
}